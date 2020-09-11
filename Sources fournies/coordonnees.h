//
// Created by prtos on 29/04/16.
// Modified by Mario, dec 2016
//

#ifndef RTC_COORDONNEES_H
#define RTC_COORDONNEES_H

#include <cmath>
#include <exception>
#include <stdexcept>
#include <iostream>
#include <sstream>

/*!
 * \class Coordonnees
 * \brief Cette classe permet de représenter les coordonnées GPS d'un endroit
 */
class Coordonnees {

public:

    Coordonnees(double latitude, double longitude);
    Coordonnees(const Coordonnees & rhs);
    double getLatitude() const ;
    double getLongitude() const ;
    static bool is_valide_coord(double p_latitude, double p_longitude) ;
    double operator- (const Coordonnees & other) const;
    friend std::ostream & operator<<(std::ostream & flux, const Coordonnees & p_coord);

private:
    double m_latitude;
    double m_longitude;
};


#endif //RTC_COORDONNEES_H
