//=======================================================================
// Copyright Lars Mescheder 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at
//  http://opensource.org/licenses/MIT)
//=======================================================================

// Implementations to ComponentTreeParser.h

template <typename G, typename A>
typename ComponentTreeParser<G,A>::Result ComponentTreeParser<G,A>::parse_(
    typename ComponentTreeParser<G,A>::GraphAccessor& graph,
    typename ComponentTreeParser<G,A>::Analyzer& analyzer,
    typename ComponentTreeParser<G,A>::PriorityQueue& boundary_nodes) {

    /// Initialization
    auto component_stack = ComponentStack{graph, analyzer};
    auto current_node = graph.get_source();
    bool flowingdown_phase = true;

    /// Parse
    // We are done, when there is no boundary node left.
    while (true) {
        // Explore neighborhood of current node.
        // The accessor has to make sure, that we access every node only once.
        while (auto neighbor_or_none = graph.get_next_neighbor(current_node)) {
            auto neighbor_node = *neighbor_or_none;
            // Flow (further) down?
            if (graph.less(graph.value(neighbor_node), graph.value(current_node))) {
                flowingdown_phase = true;
                boundary_nodes.push(current_node, graph.value(current_node));
                current_node = neighbor_node;
            } else {
                boundary_nodes.push(neighbor_node, graph.value(neighbor_node));
            }
        }

        // All neighbors already added.

        // New minimum found?
        if (flowingdown_phase) {
            component_stack.push_component(current_node, graph.value(current_node));
            flowingdown_phase = false;
        } else {
            component_stack.push_node(current_node);
        }

        auto current_node_or_none = boundary_nodes.pop();

        // Are we done?
        if (!current_node_or_none)
            break;

        current_node = *current_node_or_none;
        // Process component stack
        component_stack.raise_level(graph.value(current_node));
    }

    return analyzer.get_result();
}

template <typename G, typename A>
void ComponentTreeParser<G,A>::ComponentStack::raise_level(ComponentTreeParser<G,A>::Value new_level) {
    while (graph_.less(level(0), new_level)) {
        // level of second last component (exists, since current_level < inf)
        if  (components_.size() == 1 || graph_.less(new_level, level(1))) {
            analyzer_.raise_level(component(0), new_level);
        } else {
            assert(graph_.less(level(0), level(1)));
            // is it correct to use level instead of next_level here?
            analyzer_.merge_component_into(component(0), component(1));
            components_.pop_back();
        }
    }
}
