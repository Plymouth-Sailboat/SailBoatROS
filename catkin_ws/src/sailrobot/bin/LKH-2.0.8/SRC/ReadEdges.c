#include "LKH.h"

typedef struct edge {
    int to;
    struct edge *next; 
} edge;

typedef struct node {
    edge *first_edge;
} node;

int ReadEdges(int MaxCandidates)
{
    FILE *EdgeFile = 0;
    Node *From, *To;
    int Dimension, Edges, i, f, d, Count = 0, from, to, cost;
    int concorde_edge_format = 0;
    node *node_set;
    edge *e, *e_next;
    char line[81];

    if (EdgeFiles == 0 ||
        (EdgeFiles == 1 &&
         !(EdgeFile = fopen(CandidateFileName[0], "r"))))
        return 0;
    Dimension = ProblemType != ATSP ? DimensionSaved : 2 * DimensionSaved;
    assert(node_set = (node *) calloc(Dimension + 1, sizeof(node)));
    for (f = 0; f < EdgeFiles; f++) {
        if (EdgeFiles >= 2 &&
            !(EdgeFile = fopen(EdgeFileName[f], "r")))
            eprintf("Cannot open EDGE_FILE: \"%s\"", EdgeFileName[f]);
        if (TraceLevel >= 1)
            printff("Reading EDGE_FILE: \"%s\" ... ",
                    EdgeFileName[f]);
        fscanf(EdgeFile, "%d %d\n", &i, &Edges);
        if (i != Dimension)
            eprintf("EDGE_FILE \"%s\" does not match problem",
                    EdgeFileName[f]);
        for (i = 0; i < Edges; i++) {
            fgets(line, 80, EdgeFile);
            Count = sscanf(line, "%d %d %d\n", &from, &to, &cost);
            if (i == 0)
                concorde_edge_format = Count == 3;
            if (concorde_edge_format ? Count != 3 : Count != 2)
                eprintf("EDGE_FILE \"%s\": Wrong format\n%s",
                        EdgeFileName[f], line);
            from++;
            assert(from >= 1 && from <= Dimension);
            to++;
            assert(to >= 1 && to <= Dimension);
            From = &NodeSet[from];
            To = &NodeSet[to];
            if (concorde_edge_format && cost != Distance(From, To))
                eprintf("EDGE_FILE \"%s\": cost != Distance\n%s",
                        EdgeFileName[f], line);
            assert(e = (edge *) malloc(sizeof(edge)));
            e->to = to;
            e->next = node_set[from].first_edge;
            if (i == 0)
                assert(e->next == 0);
            node_set[from].first_edge = e;
        }
        for (from = 1; from <= Dimension; from++) {
            From = &NodeSet[from];
            for (e = node_set[from].first_edge; e; e = e->next) {
                To = &NodeSet[e->to];
                d = D(From, To);
                AddCandidate(From, To, d, 1);
                AddCandidate(To, From, d, 1);
            }
            for (e = node_set[from].first_edge; e; e = e_next) {
                e_next = e->next;
                free(e);
            }
        }
        fclose(EdgeFile);
        if (TraceLevel >= 1)
            printff("done\n");
        for (from = 1; from <= Dimension; from++)
            node_set[from].first_edge = 0;
    }
    free(node_set);
    ResetCandidateSet();
    if (MaxCandidates > 0)
        TrimCandidateSet(MaxCandidates);
    return 1;
}
