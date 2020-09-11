//
// Produced by Mario on Dec 2016.
//

#include <iostream>
#include <ctime>

#include "DonneesGTFS.h"
#include "ReseauGTFS.h"

using namespace std;

int main()
{
    const string chemin_dossier = "RTC-8aout-1dec";
    Date today(2017, 8, 18);
    Heure now1(8, 30, 0);
//    Date today; //Le constructeur par défaut initialise la date à aujourd'hui
//    Heure now1; //Le constructeur par défaut initialise l'heure à maintenant

    Heure now2 = now1.add_secondes(72000); //on désire obtenir tous les arrêts du reste de la journée

    clock_t begin = clock();
    DonneesGTFS donnees_rtc(today, now1, now2);

    donnees_rtc.ajouterLignes(chemin_dossier + "/routes.txt");
    cout << "Nombre de lignes = " << donnees_rtc.getNbLignes() << endl;
    donnees_rtc.ajouterStations(chemin_dossier + "/stops.txt");
    cout << "Nombre de stations initiales = " << donnees_rtc.getNbStations() << endl;
    donnees_rtc.ajouterServices(chemin_dossier + "/calendar_dates.txt");
    size_t nb_services = donnees_rtc.getNbServices();
    cout << "Nombre de services = " << nb_services << endl;
    if (nb_services==0) throw logic_error("main(): On doit avoir nb_services > 0 pour continuer");
    donnees_rtc.ajouterVoyagesDeLaDate(chemin_dossier + "/trips.txt");
    donnees_rtc.ajouterArretsDesVoyagesDeLaDate(chemin_dossier + "/stop_times.txt");
    donnees_rtc.ajouterTransferts(chemin_dossier + "/transfers.txt");

    clock_t end = clock();
    cout << "Chargement des données effectué en " << double(end - begin) / CLOCKS_PER_SEC << " secondes" << endl;

    cout << "Nombre de stations ayant au moins 1 arret = " << donnees_rtc.getNbStations() << endl;
    cout << "Nombre de transferts = " << donnees_rtc.getNbTransferts() << endl;
    cout << "Nombres de voyages = " << donnees_rtc.getNbVoyages() << endl;
    cout << "Nombre d'arrets = " << donnees_rtc.getNbArrets() << endl;

    begin = clock();
    ReseauGTFS reseau_rtc(donnees_rtc);
    end = clock();
    cout << "Graphe (sans le point source et destination) a été produit en " << double(end - begin) / CLOCKS_PER_SEC << " secondes" << endl;

    cout << endl;
    cout << "=============================================" << endl;
    cout << "                  premier cas                " << endl;
    cout << "=============================================" << endl;
    cout << endl;

    Coordonnees pointOrigine(46.758029, -71.336759); //Int. Chemin ste-Foy et Quatre-Bourgeois
    Coordonnees pointDestination(46.829049, -71.248305); //Centre Videotron
//    Coordonnees pointOrigine(46.7652, -71.4932);
//    Coordonnees pointDestination(46.8737, -71.322);


    cout << "Coordonnées GPS du point d'origine: " << pointOrigine << endl;
    cout << "Coordonnées GPS du point de destination: " << pointDestination << endl;
    begin = clock();
    reseau_rtc.ajouterArcsOrigineDestination(donnees_rtc, pointOrigine, pointDestination);
    end = clock();
    cout << "Nombre d'arcs ajoutés du point origine vers une station = " << reseau_rtc.getNbArcsOrigineVersStations() << endl;
    cout << "Nombre d'arcs ajoutés d'une station vers le point destination = " << reseau_rtc.getNbArcsStationsVersDestination() << endl;
    cout << "Cet ajout au graphe a nécessité un temps d'exécution de " << double(end - begin) / CLOCKS_PER_SEC << " secondes" << endl;

    long tempsExecution(0);
    reseau_rtc.itineraire(donnees_rtc, true, tempsExecution);
    cout << endl << "Temps d'exécution de l'algorithme de plus court chemin: " << tempsExecution
         << " microsecondes" << endl;

    cout << endl;
    cout << "=============================================" << endl;
    cout << "                  deuxième cas               " << endl;
    cout << "=============================================" << endl;
    cout << endl;

    cout << "Suppression du graphe connecté aux points source et destination" << endl;
    begin = clock();
    reseau_rtc.enleverArcsOrigineDestination();
    end = clock();
    cout << "Cette suppresion a nécessité un temps d'exécution de " << double(end - begin) / CLOCKS_PER_SEC << " secondes" << endl;

    Coordonnees pointOrigine2(46.829049, -71.248305); //Centre vidéotron
    Coordonnees pointDestination2(46.758029, -71.336759); //Int. Chemin ste-Foy et Quatre-Bourgeois

    cout << "Coordonnées GPS du point d'origine: " << pointOrigine2 << endl;
    cout << "Coordonnées GPS du point de destination: " << pointDestination2 << endl;
    begin = clock();
    reseau_rtc.ajouterArcsOrigineDestination(donnees_rtc, pointOrigine2, pointDestination2);
    end = clock();
    cout << "Nombre d'arcs ajoutés du point origine vers une station = " << reseau_rtc.getNbArcsOrigineVersStations() << endl;
    cout << "Nombre d'arcs ajoutés d'une station vers le point destination = " << reseau_rtc.getNbArcsStationsVersDestination() << endl;
    cout << "Cet ajout au graphe a nécessité un temps d'exécution de " << double(end - begin) / CLOCKS_PER_SEC << " secondes" << endl;

    long tempsExecution2(0);
    reseau_rtc.itineraire(donnees_rtc, true, tempsExecution2);
    cout << endl << "Temps d'exécution de l'algorithme de plus court chemin: " << tempsExecution2
         << " microsecondes" << endl;

    return 0;
}


