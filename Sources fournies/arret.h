/*!
 * \file arret.h
 * \brief Fichier contenant la classe arret
 * \author Prudencio Tossou
 * Modified by Mario, dec 2016
 */

#ifndef RTC_ARRET_H
#define RTC_ARRET_H

#include <memory>
#include "auxiliaires.h"


/*!
* \class Arret
* \brief Classe représentant un arret
*
*  Un arret est une composante d'un voyage, c'est une opération spatio-temporelle
*  (ex: la ligne 800 effectue un arrêt à la station du desjardin à 11h32).
*  Il est important de ne confondre la station et l'arret.
*
*/
class Arret {

public:
	typedef std::shared_ptr<Arret> Ptr;  //permet le raccourcis Arret::Ptr à l'externe

	Arret(unsigned int p_station_id, const Heure & p_heure_arrivee, const Heure & p_heure_depart,
          unsigned int p_numero_sequence, const std::string & p_voyage_id);
	const Heure & getHeureArrivee() const;
	const Heure & getHeureDepart() const;
	unsigned int getNumeroSequence() const;
	unsigned int getStationId() const;
	std::string getVoyageId() const;

	bool operator< (const Arret & p_other) const;
	bool operator> (const Arret & p_other) const;
	friend std::ostream & operator<<(std::ostream & flux, const Arret & p_arret);


private:
	unsigned int m_station_id;
	Heure m_heure_arrivee;
	Heure m_heure_depart;
	unsigned int m_numero_sequence;
	std::string m_voyage_id;
};


#endif //RTC_ARRET_H
