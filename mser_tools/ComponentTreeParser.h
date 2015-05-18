#ifndef COMPONENTTREEPARSER_HPP
#define COMPONENTTREEPARSER_HPP

#include <vector>
#include <stack>
#include <queue>
#include <iostream>
#include <boost/optional.hpp>

// declarations

/** \brief Generic class to parse the component tree of a graph.
 *
 *  This class is used to parse to component tree of a graph or an image. In order to do so,
 *  it is has to be configured by the two template parameters G and A, where G is a GraphAccesssor and A
 *  a ComponentTreeAnalyzer. G is thus used to access the graph (e.g. for an image) and A to analyze the evolution and
 *  merging of the components while the level changes.
 *
 *  The GraphAccessor should provide the same interface as MatAccessor and the ComponentTreeAnalyzer should provide
 *  the same interface as MatMserAnalyzer.
 *
 *  \todo Add better descriptions of ComponentTreeAnalyzer annd GraphAccessor concepts
 */
template <typename G, typename A>
// requires GraphAccessor<G>
//      &&  ComponentAnalyzer<A>
class ComponentTreeParser {
    // interface
    public:
    using GraphAccessor = G;
    using NodeIndex = typename G::NodeIndex;
    using Node = typename G::Node;
    using Data = typename G::Data;
    using Value = typename G::Value;
    using Analyzer = A;
    using Result = typename A::Result;
    using Component = typename A::Component;

    using PriorityQueue = typename G::PriorityQueue;

    ComponentTreeParser() = default;

    /**
     * @brief Call the ComponentTreeParser to do the actual parse.
     *
     * Using this overload the GraphAccessor and the ComponentTreeAnalyzer objects are automatically constructed.
     *
     * @param data       The data to be parsed. It is given as a parameter to the GraphAccessor object.
     * @param inverted   If we should go from high values to low values instead of going from low values to
     *                   high values.
     * @return Returns the result of the ComponentTreeAnalyzer.
     */
    Result operator() (const Data& data, bool inverted=false) {
        auto graph = GraphAccessor(data, inverted);
        auto analyzer = Analyzer{};
        auto boundary_nodes = PriorityQueue{inverted};
        return parse_(graph, analyzer, boundary_nodes);
    }

    /**
     * @brief Call the ComponentTreeParser to do the actual parse.
     *
     * Using this overload the GraphAccessor object is automatically constructed.

     * @param data       The data to be parsed. It is given as a parameter to the GraphAccessor object.
     * @param analyzer   The ComponentTreeAnalyzer to be used.
     * @param inverted   If we should go from high values to low values instead of going from low values to
     *                   high values.
     * @return Returns the result of the ComponentTreeAnalyzer.
     */
    Result operator() (const Data& data, Analyzer& analyzer, bool inverted=false) {
        auto graph = GraphAccessor(data, inverted);
        analyzer.reset();
        auto boundary_nodes = PriorityQueue{inverted};
        return parse_(graph, analyzer, boundary_nodes);
    }

    /**
    * @brief Call the ComponentTreeParser to do the actual parse.
    *
    * @param graph      The GraphAccessor to be used.
    * @param analyzer   The ComponentTreeAnalyzer to be used.
    * @param inverted   If we should go from high values to low values instead of going from low values to
    *                   high values.
    * @return Returns the result of the ComponentTreeAnalyzer.
    */
    Result operator() (GraphAccessor& graph, Analyzer& analyzer, bool inverted=false) {
        graph.reset();
        analyzer.reset();
        auto boundary_nodes = PriorityQueue{inverted};
        return parse_(graph, analyzer, boundary_nodes);
    }

    /**
    * @brief Call the ComponentTreeParser to do the actual parse.
    *
    * @param graph            The GraphAccessor to be used.
    * @param analyzer         The ComponentTreeAnalyzer to be used.
    * @param boundary_nodes   The priority queue to be used. It should determine the correct parsing direction.
    * @return Returns the result of the ComponentTreeAnalyzer.
    */
    Result operator() (GraphAccessor& graph, Analyzer& analyzer, PriorityQueue& boundary_nodes) {
        graph.reset();
        analyzer.reset();
        boundary_nodes.reset();
        return parse_(graph, analyzer, boundary_nodes);
    }

    private:
    /**
     * @brief The ComponentStack implements the component stack and communicates with the ComponentTreeAnalyzer
     */
    struct ComponentStack {
        public:
        ComponentStack (const GraphAccessor& graph, Analyzer& analyzer)
            : graph_(graph), analyzer_(analyzer), components_(){
            //components_.push_back(analyzer_.add_component(graph_.inf()));
        }

        void push_component(NodeIndex node_idx, Value level) {
            assert(components_.empty() || graph_.less(level, analyzer_.get_level(components_.back())));
            components_.push_back(analyzer_.add_component(graph_.node(node_idx), level));
            //current_levels_.push_back(level);
        }
        void push_node(NodeIndex node_idx) {
            assert(analyzer_.get_level(components_.back()) == graph_.value(node_idx));
            analyzer_.add_node(graph_.node(node_idx), components_.back());
        }

        void raise_level(Value level);

       // std::vector<Value> current_levels_;
        const GraphAccessor& graph_;
        Analyzer& analyzer_;
        std::vector<Component> components_;

        Value level (unsigned int i) {
            return analyzer_.get_level(components_.rbegin()[i]);
        }

        Component& component (unsigned int i) {
            return components_.rbegin()[i];
        }
    };

    /**
     * @brief Implementation of the actual parse.
     * @param graph             The GraphAccessor to be used.
     * @param analyzer          The ComponentTreeAnalyzer to be used.
     * @param boundary_nodes    The priority queue to be used.
     * @return The result of the ComponentTreeAnalyzer.
     */
    Result parse_(GraphAccessor& graph, Analyzer& analyzer, PriorityQueue& boundary_nodes);


};

// Implementations
#include "ComponentTreeParser.hpp"


#endif // COMPONENTTREEPARSER_HPP



