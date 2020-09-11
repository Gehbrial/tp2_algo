//
// Created by Mario Marchand on 16-12-30.
//

#include "ReseauGTFS.h"
#include <sys/time.h>
#include <iterator>

using namespace std;

//détermine le temps d'exécution (en microseconde) entre tv2 et tv2
long tempsExecution(const timeval &tv1, const timeval &tv2)
{
    const long unMillion = 1000000;
    long dt_usec = tv2.tv_usec - tv1.tv_usec;
    long dt_sec = tv2.tv_sec - tv1.tv_sec;
    long dtms = unMillion * dt_sec + dt_usec;
    if (dtms < 0) throw logic_error("ReaseauGTFS::tempsExecution(): dtms doit être non négatif");
    return dtms;
}

size_t ReseauGTFS::getNbArcsOrigineVersStations() const
{
    return m_nbArcsOrigineVersStations;
}

size_t ReseauGTFS::getNbArcsStationsVersDestination() const
{
    return m_nbArcsStationsVersDestination;
}

double ReseauGTFS::getDistMaxMarche() const
{
    return distanceMaxMarche;
}

//! \brief construit le réseau GTFS à partir des données GTFS
//! \param[in] Un objet DonneesGTFS
//! \post constuit un réseau GTFS représenté par un graphe orienté pondéré avec poids non négatifs
//! \post initialise la variable m_origine_dest_ajoute à false car les points origine et destination ne font pas parti du graphe
//! \post insère les données requises dans m_arretDuSommet et m_sommetDeArret et construit le graphe m_leGraphe
ReseauGTFS::ReseauGTFS(const DonneesGTFS &p_gtfs)
: m_leGraphe(p_gtfs.getNbArrets()), m_origine_dest_ajoute(false)
{
    //Le graphe possède p_gtfs.getNbArrets() sommets, mais il n'a pas encore d'arcs
    ajouterArcsVoyages(p_gtfs);
    ajouterArcsAttentes(p_gtfs);
    ajouterArcsTransferts(p_gtfs);
}

//! \brief ajout des arcs dus aux voyages
//! \brief insère les arrêts (associés aux sommets) dans m_arretDuSommet et m_sommetDeArret
//! \throws logic_error si une incohérence est détecté lors de cette étape de construction du graphe
void ReseauGTFS::ajouterArcsVoyages(const DonneesGTFS & p_gtfs)
{
    const std::map<std::string, Voyage> & voyages = p_gtfs.getVoyages();

    if(voyages.empty()) {
        throw logic_error("Données invalides: aucun voyage trouvé");
    }
    
    // Numéro de sommet
    size_t numero_sommet = 0;

    for(auto const & voyage : voyages) {
        // Arrêts du voyage traité
        std::set<Arret::Ptr, Voyage::compArret> arrets = voyage.second.getArrets();

        if(arrets.empty()) {
            throw logic_error("Données invalides: certains voyages ne possèdent aucun arrêt");
        }

        for(auto it = arrets.begin(); it != arrets.end(); ++it) {
            Arret::Ptr arret = *it;

            // Arrêt suivant l'arrêt traité
            auto prochain_arret = std::next(it, 1);

            if(prochain_arret != arrets.end()) {
                int poids = (*prochain_arret)->getHeureArrivee() - arret->getHeureArrivee();
                m_leGraphe.ajouterArc(numero_sommet, numero_sommet + 1, (unsigned) poids);
            }

            m_arretDuSommet.push_back(arret);
            m_sommetDeArret.insert(make_pair(arret, numero_sommet));
            numero_sommet++;
        }
    }
}

//! \brief ajout des arcs dus aux attentes à chaque station
//! \throws logic_error si une incohérence est détecté lors de cette étape de construction du graphe
void ReseauGTFS::ajouterArcsAttentes(const DonneesGTFS & p_gtfs)
{
    std::map<unsigned int, Station> stations = p_gtfs.getStations();

    if(stations.empty()) {
        throw logic_error("Données invalides: aucune station trouvée");
    }

    for (auto const &station: stations) {
        // Arrêts de la station traitée
        const std::multimap<Heure, Arret::Ptr> &arrets = station.second.getArrets();

        if(arrets.empty()) {
            throw logic_error("Données invalides: certaines stations ne possèdent aucun arrêt");
        }

        for (auto it = arrets.begin(); it != arrets.end(); ++it) {
            Arret::Ptr arret = it->second;

            //Arrêt suivant l'arrêt traité
            auto prochain_arret = std::next(it, 1);

            //On a calculé le poids à partir du premier arrêt successif n'étant pas dans le même voyage
            bool poids_trouve = false;

            while (!poids_trouve && prochain_arret != arrets.end()) {
                //Si les arrêts ne font pas partie du même voyae
                if (arret->getVoyageId() != prochain_arret->second->getVoyageId()) {
                    int poids = prochain_arret->second->getHeureArrivee() - arret->getHeureArrivee();

                    if (m_sommetDeArret.count(arret) == 0)
                        throw logic_error(
                                "Tentative d'ajout d'un arc dont le sommet origine n'est pas présent dans m_sommetDeArret");

                    if (m_sommetDeArret.count(prochain_arret->second) == 0)
                        throw logic_error(
                                "Tentative d'ajout d'un arc dont le sommet destination n'est pas présent dans m_sommetDeArret");

                    m_leGraphe.ajouterArc(m_sommetDeArret.at(arret), m_sommetDeArret.at(prochain_arret->second),
                                          (unsigned) poids);
                    //On peut cesser la recherche d'un arrêt valide
                    poids_trouve = true;
                } else {
                    //On passe au prochain arrêt
                    prochain_arret = std::next(prochain_arret, 1);
                }
            }
        }
    }
}

//! \brief ajouts des arcs dus aux transferts entre stations
//! \throws logic_error si une incohérence est détecté lors de cette étape de construction du graphe
void ReseauGTFS::ajouterArcsTransferts(const DonneesGTFS & p_gtfs)
{
    std::map<unsigned int, Station> stations = p_gtfs.getStations();

    if(stations.empty()) {
        throw logic_error("Données invalides: aucune station trouvée");
    }

    std::vector<std::tuple<unsigned int, unsigned int, unsigned int>> transferts = p_gtfs.getTransferts();

    if(transferts.empty()) {
        throw logic_error("Données invalides: aucun transfert trouvé");
    }

    for (auto const & transfert : transferts) {
        //Station de départ
        Station from_station = stations.at(std::get<0>(transfert));

        //Station de destination
        Station to_station = stations.at(std::get<1>(transfert));

        //Temps de transfert
        unsigned int transfer_time = std::get<2>(transfert);

        //Arrêts de la station de départ
        const std::multimap<Heure, Arret::Ptr> &from_arrets = from_station.getArrets();

        if(from_arrets.empty()) {
            throw logic_error("Certaines stations ne possèdent aucun arrêt");
        }

        for (auto it_source = from_arrets.begin(); it_source != from_arrets.end(); ++it_source) {
            //Somme de l'heure d'arrivée de l'arrêt traité dans la station de départ et du temps de transfert
            Heure lower_bound = it_source->first.add_secondes(transfer_time);

            //Itérateur pointant vers un arrêt de la station de destination
            auto it_dest = to_station.getArrets().lower_bound(lower_bound);

            if (it_dest != to_station.getArrets().end()) {
                unsigned int poids = transfer_time + (it_dest->first - lower_bound);

                if (m_sommetDeArret.count(it_source->second) == 0)
                    throw logic_error(
                            "Tentative d'ajout d'un arc dont le sommet origine n'est pas présent dans m_sommetDeArret");

                if (m_sommetDeArret.count(it_dest->second) == 0)
                    throw logic_error(
                            "Tentative d'ajout d'un arc dont le sommet destination n'est pas présent dans m_sommetDeArret");

                m_leGraphe.ajouterArc(m_sommetDeArret.at(it_source->second), m_sommetDeArret.at(it_dest->second),
                                      poids);
            }
        }
    }
}

//! \brief ajoute des arcs au réseau GTFS à partir des données GTFS
//! \brief Il s'agit des arcs allant du point origine vers une station si celle-ci est accessible à pieds et des arcs allant d'une station vers le point destination
//! \param[in] p_gtfs: un objet DonneesGTFS
//! \param[in] p_pointOrigine: les coordonnées GPS du point origine
//! \param[in] p_pointDestination: les coordonnées GPS du point destination
//! \throws logic_error si une incohérence est détecté lors de la construction du graphe
//! \post constuit un réseau GTFS représenté par un graphe orienté pondéré avec poids non négatifs
//! \post assigne la variable m_origine_dest_ajoute à true (car les points orignine et destination font parti du graphe)
//! \post insère dans m_sommetsVersDestination les numéros de sommets connctés au point destination
void ReseauGTFS::ajouterArcsOrigineDestination(const DonneesGTFS &p_gtfs, const Coordonnees &p_pointOrigine,
                                               const Coordonnees &p_pointDestination)
{
    //Shared pointer sur le point d'origine
    Arret::Ptr ptr_point_origine = make_shared<Arret>(stationIdOrigine, Heure(0, 0, 0), Heure(0, 0, 0), 0, "");

    //Shared pointer sur le point de destination
    Arret::Ptr ptr_point_dest = make_shared<Arret>(stationIdDestination, Heure(0, 0, 0), Heure(0, 0, 0), 0, "");

    try {
        m_arretDuSommet.push_back(ptr_point_origine);
        m_sommetDeArret.insert(std::pair<Arret::Ptr, size_t>(ptr_point_origine, m_sommetDeArret.size()));
        m_sommetOrigine = m_sommetDeArret.at(ptr_point_origine);
    } catch(exception &e) {
        throw logic_error("Erreur lors de l'ajout d'un sommet d'origine");
    }

    try {
        m_arretDuSommet.push_back(ptr_point_dest);
        m_sommetDeArret.insert(std::pair<Arret::Ptr, size_t>(ptr_point_dest, m_sommetDeArret.size()));
        m_sommetDestination = m_sommetDeArret.at(ptr_point_dest);
    } catch(exception &e) {
        throw logic_error("Erreur lors de l'ajout d'un sommet destination");
    }

    //Redimension du graphe pour accomoder les nouveaux sommets
    m_leGraphe.resize(m_leGraphe.getNbSommets() + 2);

    //ajout des arcs à pieds entre le point source et les arrets des stations atteignables
    m_nbArcsOrigineVersStations = 0;

    std::map<unsigned int, Station> stations = p_gtfs.getStations();

    if(stations.empty()) {
        throw logic_error("Données invalides: aucune station trouvée");
    }

    for (auto const &station: stations) {
        //Distance entre le point d'origine et la station traitée
        double dist_station = station.second.getCoords() - p_pointOrigine;

        //Si la station est atteignable à pied
        if (dist_station <= distanceMaxMarche) {
            //Temps en secondes pour se rendre à la station
            unsigned int temps_marche = (unsigned int) ((dist_station / vitesseDeMarche) * 3600);

            //Heure d'arrivée à la station
            Heure heure_arrivee = p_gtfs.getTempsDebut().add_secondes(temps_marche);

            //Itérateur pointant sur un arrêt de la station
            auto it_dest = station.second.getArrets().lower_bound(heure_arrivee);

            if (it_dest != station.second.getArrets().end()) {
                unsigned int poids = temps_marche + (it_dest->first - heure_arrivee);

                if (m_sommetDeArret.count(it_dest->second) == 0)
                    throw logic_error(
                            "Tentative d'ajout d'un arc dont le sommet destination n'est pas présent dans m_sommetDeArret");

                m_leGraphe.ajouterArc(m_sommetOrigine, m_sommetDeArret.at(it_dest->second), poids);
                m_nbArcsOrigineVersStations++;
            }
        }
    }

    //ajout des arcs à pieds des arrêts de certaine stations vers l'arret point destination
    m_nbArcsStationsVersDestination = 0;

    for (auto const &station: stations) {
        //Distance entre la station et la point de destination
        double dist_dest = p_pointDestination - station.second.getCoords();

        //Si la station est atteignable à pied
        if (dist_dest <= distanceMaxMarche) {
            //Le poids est égal au temps de marche pour se rendre à la station à pied
            unsigned int poids = (unsigned int) ((dist_dest / vitesseDeMarche) * 3600);

            //Arrêts de la station traitée
            const std::multimap<Heure, Arret::Ptr> &arrets = station.second.getArrets();

            if(arrets.empty()) {
                throw logic_error("Certaines stations ne possèdent aucun arrêt");
            }

            for (auto it = arrets.begin(); it != arrets.end(); ++it) {
                Arret::Ptr arret = it->second;

                if (m_sommetDeArret.count(arret) == 0)
                    throw logic_error(
                            "Tentative d'ajout d'un arc dont le sommet origine n'est pas présent dans m_sommetDeArret");

                size_t sommet = m_sommetDeArret.at(arret);

                m_leGraphe.ajouterArc(sommet, m_sommetDestination, poids);
                m_sommetsVersDestination.push_back(sommet);
                m_nbArcsStationsVersDestination++;
            }
        }
    }

    m_origine_dest_ajoute = true;
}

//! \brief Remet ReseauGTFS dans l'était qu'il était avant l'exécution de ReseauGTFS::ajouterArcsOrigineDestination()
//! \param[in] p_gtfs: un objet DonneesGTFS
//! \throws logic_error si une incohérence est détecté lors de la modification du graphe
//! \post Enlève de ReaseauGTFS tous les arcs allant du point source vers un arrêt de station et ceux allant d'un arrêt de station vers la destination
//! \post assigne la variable m_origine_dest_ajoute à false (les points orignine et destination sont enlevés du graphe)
//! \post enlève les données de m_sommetsVersDestination
void ReseauGTFS::enleverArcsOrigineDestination()
{
    //On enlève tous les sommets contenus dans m_sommetsVersDestination
    for (auto const & sommet : m_sommetsVersDestination) {
        m_leGraphe.enleverArc(sommet, m_sommetDestination);
    }

    //On enlève du graphe les sommets associés au point d'origine et au point de destination
    if(m_leGraphe.getNbSommets() >= 2)
        m_leGraphe.resize(m_leGraphe.getNbSommets() - 2);
    else
        m_leGraphe.resize(0);

    //On enlève les sommets d'origine et de destination dans m_sommetDeArret
    try {
        m_sommetDeArret.erase(m_sommetDeArret.find(m_arretDuSommet[m_sommetOrigine]));
        m_sommetDeArret.erase(m_sommetDeArret.find(m_arretDuSommet[m_sommetDestination]));
    } catch(exception &e) {
        throw logic_error("Erreur lors de la réinitialisation de m_sommetDeArret");
    }

    //On enlève les sommets d'origine et de destination dans m_arretDuSommet
    try {
        m_arretDuSommet.erase(m_arretDuSommet.begin() + m_sommetOrigine);
        m_arretDuSommet.erase(m_arretDuSommet.begin() + m_sommetDestination);
    } catch(exception &e) {
        throw logic_error("Erreur lors de la réinitialisation de m_arretDuSommet");
    }

    m_nbArcsOrigineVersStations = 0;
    m_nbArcsStationsVersDestination = 0;
    m_origine_dest_ajoute = false;
}


//! \brief Trouve le plus court chemin menant du point d'origine au point destination préalablement choisis
//! \brief Permet également d'affichier l'itinéraire du voyage et retourne le temps d'exécution de l'algorithme de plus court chemin utilisé
//! \param[in] p_afficherItineraire: true si on désire afficher l'itinéraire et false autrement
//! \param[out] p_tempsExecution: le temps d'exécution de l'algorithme de plus court chemin utilisé
//! \throws logic_error si un problème survient durant l'exécution de la méthode
void ReseauGTFS::itineraire(const DonneesGTFS &p_gtfs, bool p_afficherItineraire, long &p_tempsExecution) const
{
    if (!m_origine_dest_ajoute)
        throw logic_error(
                "ReseauGTFS::afficherItineraire(): il faut ajouter un point origine et un point destination avant d'obtenir un itinéraire");

    vector<size_t> chemin;

    timeval tv1;
    timeval tv2;
    if (gettimeofday(&tv1, 0) != 0)
        throw logic_error("ReseauGTFS::afficherItineraire(): gettimeofday() a échoué pour tv1");
    unsigned int tempsDuTrajet = m_leGraphe.plusCourtChemin(m_sommetOrigine, m_sommetDestination, chemin);
    if (gettimeofday(&tv2, 0) != 0)
        throw logic_error("ReseauGTFS::afficherItineraire(): gettimeofday() a échoué pour tv2");
    p_tempsExecution = tempsExecution(tv1, tv2);

    if (tempsDuTrajet == numeric_limits<unsigned int>::max())
    {
        if (p_afficherItineraire)
            cout << "La destination n'est pas atteignable de l'orignine durant cet intervalle de temps" << endl;
        return;
    }

    if (tempsDuTrajet == 0)
    {
        if (p_afficherItineraire) cout << "Vous êtes déjà situé à la destination demandée" << endl;
        return;
    }

    //un chemin non trivial a été trouvé
    if (chemin.size() <= 2)
        throw logic_error("ReseauGTFS::afficherItineraire(): un chemin non trivial doit contenir au moins 3 sommets");
    if (m_arretDuSommet[chemin[0]]->getStationId() != stationIdOrigine)
        throw logic_error("ReseauGTFS::afficherItineraire(): le premier noeud du chemin doit être le point origine");
    if (m_arretDuSommet[chemin[chemin.size() - 1]]->getStationId() != stationIdDestination)
        throw logic_error(
                "ReseauGTFS::afficherItineraire(): le dernier noeud du chemin doit être le point destination");

    if (p_afficherItineraire)
    {
        std::cout << std::endl;
        std::cout << "=====================" << std::endl;
        std::cout << "     ITINÉRAIRE      " << std::endl;
        std::cout << "=====================" << std::endl;
        std::cout << std::endl;
    }

    if (p_afficherItineraire) cout << "Heure de départ du point d'origine: "  << p_gtfs.getTempsDebut() << endl;
    Arret::Ptr ptr_a = m_arretDuSommet.at(chemin[0]);
    Arret::Ptr ptr_b = m_arretDuSommet.at(chemin[1]);
    if (p_afficherItineraire)
        cout << "Rendez vous à la station " << p_gtfs.getStations().at(ptr_b->getStationId()) << endl;

    unsigned int sommet = 1;

    while (sommet < chemin.size() - 1)
    {
        ptr_a = ptr_b;
        ++sommet;
        ptr_b = m_arretDuSommet.at(chemin[sommet]);
        while (ptr_b->getStationId() == ptr_a->getStationId())
        {
            ptr_a = ptr_b;
            ++sommet;
            ptr_b = m_arretDuSommet.at(chemin[sommet]);
        }
        //on a changé de station
        if (ptr_b->getStationId() == stationIdDestination) //cas où on est arrivé à la destination
        {
            if (sommet != chemin.size() - 1)
                throw logic_error(
                        "ReseauGTFS::afficherItineraire(): incohérence de fin de chemin lors d'un changement de station");
            break;
        }
        if (sommet == chemin.size() - 1)
            throw logic_error("ReseauGTFS::afficherItineraire(): on ne devrait pas être arrivé à destination");
        //on a changé de station mais sommet n'est pas le noeud destination
        string voyage_id_a = ptr_a->getVoyageId();
        string voyage_id_b = ptr_b->getVoyageId();
        if (voyage_id_a != voyage_id_b) //on a changé de station à pieds
        {
            if (p_afficherItineraire)
                cout << "De cette station, rendez-vous à pieds à la station " << p_gtfs.getStations().at(ptr_b->getStationId()) << endl;
        }
        else //on a changé de station avec un voyage
        {
            Heure heure = ptr_a->getHeureArrivee();
            unsigned int ligne_id = p_gtfs.getVoyages().at(voyage_id_a).getLigne();
            string ligne_numero = p_gtfs.getLignes().at(ligne_id).getNumero();
            if (p_afficherItineraire)
                cout << "De cette station, prenez l'autobus numéro " << ligne_numero << " à l'heure " << heure << " "
                     << p_gtfs.getVoyages().at(voyage_id_a) << endl;
            //maintenant allons à la dernière station de ce voyage
            ptr_a = ptr_b;
            ++sommet;
            ptr_b = m_arretDuSommet.at(chemin[sommet]);
            while (ptr_b->getVoyageId() == ptr_a->getVoyageId())
            {
                ptr_a = ptr_b;
                ++sommet;
                ptr_b = m_arretDuSommet.at(chemin[sommet]);
            }
            //on a changé de voyage
            if (p_afficherItineraire)
                cout << "et arrêtez-vous à la station " << p_gtfs.getStations().at(ptr_a->getStationId()) << " à l'heure "
                     << ptr_a->getHeureArrivee() << endl;
            if (ptr_b->getStationId() == stationIdDestination) //cas où on est arrivé à la destination
            {
                if (sommet != chemin.size() - 1)
                    throw logic_error(
                            "ReseauGTFS::afficherItineraire(): incohérence de fin de chemin lors d'u changement de voyage");
                break;
            }
            if (ptr_a->getStationId() != ptr_b->getStationId()) //alors on s'est rendu à pieds à l'autre station
                if (p_afficherItineraire)
                    cout << "De cette station, rendez-vous à pieds à la station " << p_gtfs.getStations().at(ptr_b->getStationId()) << endl;
        }
    }

    if (p_afficherItineraire)
    {
        cout << "Déplacez-vous à pieds de cette station au point destination" << endl;
        cout << "Heure d'arrivée à la destination: " << p_gtfs.getTempsDebut().add_secondes(tempsDuTrajet) << endl;
    }
    unsigned int h = tempsDuTrajet / 3600;
    unsigned int reste_sec = tempsDuTrajet % 3600;
    unsigned int m = reste_sec / 60;
    unsigned int s = reste_sec % 60;
    if (p_afficherItineraire)
    {
        cout << "Durée du trajet: " << h << " heures, " << m << " minutes, " << s << " secondes" << endl;
    }

}


