#include "ComponentTreeParser.hpp"

struct G {
    using NodeIndex = int;
    using Value = int;

    static const Value inf = 1000;


    void reset_status () {}
    NodeIndex get_source () { return 0; }
    bool note_is_saturated () { return true; }
    NodeIndex get_next_neighbor (NodeIndex node) { return node; }

    Value operator[] (NodeIndex node) { return 0; }
};

struct A {
    using ComponentRef = int;
    using Result = int;

    ComponentRef add_node( int node, int value) { return 0; }
    ComponentRef add_component () { return 0; }
    ComponentRef merge_components (ComponentRef comp1, ComponentRef comp2) {return 0;}

};

void test() {
    G graph;
    ComponentTreeParser<G, A> test;
    test(graph);
}



