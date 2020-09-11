//
//  Graphe.h
//  Classe pour graphes orientés pondérés (non négativement) avec listes d'adjacence
//
//  Mario Marchand automne 2016.
//

#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <list>
#include <stack>
#include <limits>
#include <iostream>
#include <algorithm>

//! \brief  Classe pour graphes orientés pondérés (non négativement) avec listes d'adjacence
class Graphe
{
public:

	Graphe(size_t = 0);
    void resize(size_t);
	void ajouterArc(size_t i, size_t j, unsigned int poids);
	void enleverArc(size_t i, size_t j);
	unsigned int getPoids(size_t i, size_t j) const;
	size_t getNbSommets() const;

    unsigned int plusCourtChemin(size_t p_origine, size_t p_destination,
                             std::vector<size_t> & p_chemin) const;

private:

	struct Arc
	{
		Arc(size_t dest, unsigned int p) :
				destination(dest), poids(p)
		{
		}
		size_t destination;
		unsigned int poids;
	};

	std::vector<std::list<Arc> > m_listesAdj; /*!< les listes d'adjacence */

};

#endif  //GRAPH_H
