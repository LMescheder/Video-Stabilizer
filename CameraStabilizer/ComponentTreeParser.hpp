#ifndef COMPONENTTREEPARSER_HPP_
#define COMPONENTTREEPARSER_HPP_

#include <vector>
#include <stack>
#include <queue>
#include <utility>


/*
G must provide:
  Node
  Value - must be totally ordered by <
  Data

  static const Value inf

  G(const Data&)
  Node get_source()

  Node must provide:
    Value value ()
    bool accessible ()
    void set_accessible (bool accessible)

    bool has_more_neighbors()
    Node get_next_neighbor()

A must provide:
  Component
  Result

  add_node(G::Node, G::Value)

  ComponentRef add_component()
  merge_components(ComponentRef comp1, ComponentRef comp2)
*/

// declaration
template <typename G, typename A>
class ComponentTreeParser {
    // interface
    public:
    using GraphAccessor = G;
    using Value = typename G::Value;
    using Node = typename G::Node;
    using Data = typename G::Data;

    using Analyzer = A;
    using Result = typename A::Result;
    using Component = typename A::Component;

    ComponentTreeParser () = default;

    Result operator() (const Data& data) {
        return compute_(data);
    }

    // implementation
    private:

    struct ComponentStack {
        public:
        ComponentStack (Analyzer analyzer) : analyzer_(analyzer) {}

        void push_component(Value level) {
            components_.push_back(analyzer_.add_component());
        }
        void push_node(Node node) {
            analyzer_.add_node(node, components_.back());
        }

        void raise_level(Value level) {
            Value current_level = components_.rbegin()[0].level();  // level of last component
            Value next_level = components_.rbegin()[1].level();     // level of second last component

            while (level > current_level) {
                if (level < next_level) {
                    components_.back().set_level(level);
                } else {
                    auto current_component = components_.back();
                    components_.pop_back();
                    auto next_component = components_.back();
                    components_.pop_back();
                    next_level = components_.back().level;
                    components_.push_back(analyzer_.merge_components(current_component, next_component));
                    current_level = components_.back().level;
                }
            }
        }

        private:
        std::vector<Component> components_;
        Analyzer analyzer_;
    };

    struct NodeLess {
        bool operator() (const Node& node1, const Node& node2) {
            return (node1.value() < node2.value());
        }
    };

    // actual algorithm
    Result compute_(const Data& data) {
        // data structures
        GraphAccessor graph{data};
        Analyzer analyzer;
        ComponentStack component_stack;
        std::priority_queue<Node, std::vector<Node>, NodeLess> boundary_nodes;

        // push dummy component on the stack
        component_stack.push_component(G::inf);

        // get entrance point and intialize flowing down phase
        Node current_node = graph.get_source();;
        bool flowingdown_phase = true;

        // we are done, when there is no boundary node left
        while (!boundary_nodes.empty()) {
            // explore neighborhood of current node
            while (current_node.has_more_neighbors()) {
                Node neighbor_node = current_node.get_next_neighbor();
                if (!neighbor_node.accessible()) {
                    neighbor_node.set_accessible(true);

                    // flow (further) down?
                    if (neighbor_node.value() < current_node.value()) {
                        // yes, flow (further) down!
                        flowingdown_phase = true;
                        boundary_nodes.push(current_node);
                        current_node = neighbor_node;
                    } else {
                        // no, stay here and look at other neighbors
                        boundary_nodes.push(neighbor_node);
                    }
                }
            }
            // all neighboring nodes explored, now process components
            if (flowingdown_phase) {
                // end flowing down phase, new minimum found
                flowingdown_phase = false;
                component_stack.push_component(current_node.value());
            }
            component_stack.push_node(current_node);

            current_node = boundary_nodes.top();
            boundary_nodes.pop();
            // raise level of current component
            component_stack.raise_level(current_node.value());
        }
    }
};

#endif // COMPONENTTREEPARSER_HPP_
