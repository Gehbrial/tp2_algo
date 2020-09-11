//
// Created by prtos on 29/04/16.
// Modified by Mario, Dec, 2016
//

#ifndef RTC_VOYAGE_H
#define RTC_VOYAGE_H

#include <string>
#include <set>
#include <memory>
#include "arret.h"
#include "auxiliaires.h"

/*!
 * \class Voyage
 * \brief Classe permettant de décrire un voyage
 * \note Sa déclaration nécessite la déclaration partielle des classe Ligne et Station, en raison de références circulaires avec ces classes.
 */
class Voyage {

public:

    struct compArret //foncteur de comparaison pour m_arrets
    {
        bool operator() (Arret::Ptr i, Arret::Ptr j) const;
    };

    Voyage(const std::string & p_id, unsigned int p_ligne_id, const std::string & p_service_id, const std::string & p_destination);
    Voyage();
	const std::set<Arret::Ptr, compArret> & getArrets() const;
    unsigned int getNbArrets() const;
	const std::string& getDestination() const;
	std::string getId() const;
	unsigned int getLigne() const;
	std::string getServiceId() const;
	Heure getHeureDepart() const;
	Heure getHeureFin() const;
    void ajouterArret(const Arret::Ptr & p_arret);
	bool operator< (const Voyage & p_other) const;
	bool operator> (const Voyage & p_other) const;
	friend std::ostream & operator<<(std::ostream & flux, const Voyage & p_voyage);

private:

    std::string m_id;
	unsigned int m_ligne;
	std::string m_service_id;
	std::string m_destination;
	std::set<Arret::Ptr, compArret> m_arrets;

};


#endif //RTC_VOYAGE_H
