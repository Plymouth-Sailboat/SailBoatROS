#include "LKH.h"
#include "gpx.h"

/*
 * The MergeWithTourGPX2 function attempts to find a short tour
 * by merging a given tour, T1, with another tour, T2.
 * T1 is given by the Suc pointers of its nodes.
 * T2 is given by the Next pointers of its nodes.
 *
 * The merging algorithm uses Generalized Partition Crossover 2,
 * GPX2, described in
 *
 *      R.Tinos, D. Whitley, and G. Ochoa (2017),
 *      A new generalized partition crossover for the traveling
 *      salesman problem: tunneling between local optima.
 */

GainType MergeWithTourGPX2()
{
    int *red, *blue, *offspring, i;
    GainType Cost1 = 0, Cost2 = 0, NewCost;
    Node *N;

    n_cities = Dimension;
    red = (int *) malloc(n_cities * sizeof(int));
    blue = (int *) malloc(n_cities * sizeof(int));
    offspring = (int *) malloc((n_cities + 1) * sizeof(int));
    Map2Node = (Node **) malloc(n_cities * sizeof(Node *));

    N = FirstNode;
    i = 0;
    do {
        Map2Node[i] = N;
        red[i] = N->Rank = i;
        i++;
        Cost1 += C(N, N->Suc) - N->Pi - N->Suc->Pi;
    } while ((N = N->Suc) != FirstNode);
    i = 0;
    do {
        blue[i++] = N->Rank;
        Cost2 += C(N, N->Next) - N->Pi - N->Next->Pi;
    } while ((N = N->Next) != FirstNode);
    Cost1 /= Precision;
    Cost2 /= Precision;

    NewCost = gpx(red, blue, offspring);
    if (NewCost >= Cost1 || NewCost >= Cost2) {
        free(red);
        free(blue);
        free(offspring);
        return Cost1;
    }
    offspring[n_cities] = offspring[0];
    for (i = 0; i < n_cities; i++) {
        N = Map2Node[offspring[i]];
        Node *NextN = Map2Node[offspring[i + 1]];
        N->Suc = NextN;
        NextN->Pred = N;
    }
    Hash = 0;
    N = FirstNode;
    do
        Hash ^= Rand[N->Id] * Rand[N->Suc->Id];
    while ((N = N->Suc) != FirstNode);
    free(red);
    free(blue);
    free(offspring);
    free(Map2Node);
    if (TraceLevel >= 2)
        printff("GPX2: " GainFormat "\n",  NewCost);
    return NewCost;
}
