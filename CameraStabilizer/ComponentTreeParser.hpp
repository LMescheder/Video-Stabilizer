#ifndef COMPONENTTREEPARSER_HPP_
#define COMPONENTTREEPARSER_HPP_

#include <vector>
#include <stack>
#include <queue>
#include <utility>


/*
G must provide:
  G::NodeIndex
  G::Value - must be totally ordered by <

  G::Value G::inf

  void G::reset_status()
  G::NodeIndex G::get_source()
  bool G::note_saturated(G::NodeIndex node)
  G::NodeIndex G::get_next_neighbor(G::NodeIndex node)

  bool G::note_accessible (G::NodeIndex node)
  void G::set_note_accessible (G::NodeIndex node, bool accessible)

  G::Value G::operator[] (G::NodeIndex node)


A must provide:
  A::ComponentRef
  A::Result

  A::ComponentRef::add_node(G::Node, G::Value)

  A::ComponentRef A<G>::add_component()
  A::ComponentRef A<G>::merge_components(A<G>::ComponentRef comp1, A<G>::ComponentRef comp2)
*/

// declaration
template <typename G, typename A>
class ComponentTreeParser {
    public:
    using Graph = G;
    using Value = typename G::Value;
    using NodeIndex = typename G::NodeIndex;

    using Analyzer = A;
    using Result = typename A::Result;
    using ComponentRef = typename A::ComponentRef;
    // constructor <- necessary parameters good be added
    ComponentTreeParser () = default;

    // operator
    Result operator() (G& graph);

    private:
    struct Component {
        Value value;
        std::vector<Value> nodes;
        ComponentRef compref;
    };


};


template <typename T1, typename T2>
struct FirstArgumentLess  {
    bool operator() (const std::pair<T1, T2>& pair1, const std::pair<T1, T2>& pair2) { return pair1.first < pair2.first; }
};

// implementation
template <typename G, typename A>
typename A::Result ComponentTreeParser<G, A>::operator() (G& graph) {
    // abbreviation
    using NodeValuePair = std::pair<Value, NodeIndex>;

    // analyzer, which examines the components
    Analyzer analyzer;

    // data structures
    std::stack<Component> component_stack;
    std::priority_queue<NodeValuePair, std::vector<NodeValuePair>, FirstArgumentLess<Value, NodeIndex>> boundary_nodes;

    NodeIndex current_node;
    Value current_level;

    // initialize: reset status and push dummy component to the stack
    graph.reset_status();
    component_stack.push(Component{G::inf, {}, analyzer.add_component()});

    // get current node and its level
    current_node = graph.get_source();
    current_level = graph[current_node];

    // flowing down phase
    while (!graph.is_saturated(node)) {
        NodeIndex next_node;
        do {
            next_node = graph.get_next_neighbor(node);
        } while (!graph.node_accessible(next_node));
    }

}
#endif // COMPONENTTREEPARSER_HPP_
