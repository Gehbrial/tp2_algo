//
// Created by prtos on 13/06/16.
// Modified by Mario, dec 2016
//

#ifndef RTC_AUXILIAIRES_H
#define RTC_AUXILIAIRES_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include "time.h"
#include <unordered_set>
#include <algorithm>

/*!
 * \class Date
 * \brief Cette classe représente une date.
 */
class Date
{

public:
    Date();
    Date(unsigned int an, unsigned int mois, unsigned int jour);
    bool operator==(const Date &other) const;
    bool operator<(const Date &other) const;
    bool operator>(const Date &other) const;
    friend std::ostream &operator<<(std::ostream &flux, const Date &p_date);


private:
    int m_code; // nombre de jours depuis 1970-01-01 pour la date en parametre
    unsigned int m_an;
    unsigned int m_mois;
    unsigned int m_jour;

    void encode(unsigned int an, unsigned int mois, unsigned int jour);

};


/*!
 * \class Heure
 * \brief Cette classe représente l'heure d'une journée.
 * Cependant pour les besoins du travail pratique nous permettont qu'elle puisse encoder un nombre d'heures supérieurs à 24
 */
class Heure
{
public:
    Heure();

    Heure(unsigned int heure, unsigned int min, unsigned int sec);
    Heure add_secondes(unsigned int secs) const;
    bool operator==(const Heure &other) const;
    bool operator<(const Heure &other) const;
    bool operator>(const Heure &other) const;
    bool operator<=(const Heure &other) const;
    bool operator>=(const Heure &other) const;
    int operator-(const Heure &other) const;
    friend std::ostream &operator<<(std::ostream &flux, const Heure &p_heure);

private:
    unsigned int m_code;
    unsigned int m_heure;
    unsigned int m_min;
    unsigned int m_sec;

    void encode(unsigned int heure, unsigned int min, unsigned int sec);
};


#endif //RTC_AUXILIAIRES_H
