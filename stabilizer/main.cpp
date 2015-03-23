#include "ComponentTreeParser.hpp"

struct G {
    using Value = int;
    using Data = int;
    struct NodeIndex {
        Value value = 0;
    };
    bool has_more_neighbors(NodeIndex node) { return false; }
    NodeIndex get_next_neighbor(NodeIndex node) { return node; }
    Value value(NodeIndex node) {return node.value;}
    static const Value inf = 1000;

    G (const Data&) {}


    NodeIndex get_source () { return NodeIndex{}; }

};


struct A {
    struct Component {
        int level_ = 0;
        int level() {return level_;}
        void set_level(int level) {level_ = level;}
    };
    using Result = int;

    void add_node( G::NodeIndex node, Component component) { }
    Component add_component () { return Component{}; }
    Component merge_components (Component comp1, Component comp2) {return Component{};}


    Result get_result() {return Result{}; }
};

int main() {
    int data = 0;
    ComponentTreeParser<G, A> test;
    test(data);
}



