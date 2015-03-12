#include "ComponentTreeParser.hpp"

struct G {
    using Value = int;
    struct Node {
        Value value () const { return 0; }
        bool accessible () { return true; }
        void set_accessible (bool accessible) {}

        bool has_more_neighbors() { return false; }
        Node get_next_neighbor() { return *this; }

    };
    static const Value inf = 1000;



    void reset_status () {}
    Node get_source () { return Node{}; }

};

struct A {
    using ComponentRef = int;
    using Result = int;

    ComponentRef add_node( G::Node node, G::Value value) { return 0; }
    ComponentRef add_component () { return 0; }
    ComponentRef merge_components (ComponentRef comp1, ComponentRef comp2) {return 0;}

};

void test() {
    G graph;
    ComponentTreeParser<G, A> test;
    test(graph);
}



