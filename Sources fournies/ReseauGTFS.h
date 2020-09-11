//
// Created by Mario Marchand on 16-12-30.
//

#ifndef TP2_RESEAUGTFS_H
#define TP2_RESEAUGTFS_H

#include "DonneesGTFS.h"
#include "graphe.h"


class ReseauGTFS
{

public:
    ReseauGTFS(const DonneesGTFS &);
    void ajouterArcsOrigineDestination(const DonneesGTFS &, const Coordonnees &, const Coordonnees &);
    void enleverArcsOrigineDestination();
    void itineraire(const DonneesGTFS &, bool, long &) const;
    size_t getNbArcsOrigineVersStations() const;
    size_t getNbArcsStationsVersDestination() const;
    double getDistMaxMarche() const;

private:
    Graphe m_leGraphe;
    std::vector<Arret::Ptr> m_arretDuSommet; //m_arretDuSommet[i] est le pointeur (shared_ptr) de l'arret (associé au sommet i du graphe
    std::unordered_map<Arret::Ptr,size_t> m_sommetDeArret; //m_sommetDeArret[a_ptr] est le sommet du graphe associé au pointeur de l'arret a_ptr
    std::vector<size_t> m_sommetsVersDestination; //Chaque élément est un sommet possédant un arc vers la destination

    bool m_origine_dest_ajoute; //indique si on a ajouté le point origine, le point destination, et les arcs correspondants
    size_t m_sommetOrigine; //le sommet du graphe qui représente le point d'origine
    size_t m_sommetDestination; //le sommet du graphe qui représente le point destination
    size_t m_nbArcsOrigineVersStations; //le nombre d'arcs du point origine vers des stations
    size_t m_nbArcsStationsVersDestination; //le nombre d'arcs d'une station vers le point destination

    const double vitesseDeMarche = 5.0; // vitesse moyenne de marche, en km/heure, d'un humain selon wikipedia */
    const double distanceMaxMarche = 1.5; // distance maximale de marche permise, en km
    const unsigned int stationIdOrigine = 0; //numéro de stationID donné pour l'arret fantôme de départ
    const unsigned int stationIdDestination = 1; //numéro de stationID donné pour les arrets fantômes de destination

    void ajouterArcsVoyages(const DonneesGTFS &); //ajout des arcs dus aux voyages
    void ajouterArcsAttentes(const DonneesGTFS &); //ajout des arcs dus aux attentes à une station (arcs temporels)
    void ajouterArcsTransferts(const DonneesGTFS &); //ajout des arcs dus aux transferts
};


#endif //TP2_RESEAUGTFS_H
