#ifndef CAMERASTABILIZER_TREE_H_
#define CAMERASTABILIZER_TREE_H_

#include <vector>

template <typename T>
class Tree {
    public:
    struct Node;


    private:
    std::vector<Node> nodes;
};

template <typename T>
struct Tree<T>::Node {
    Node* left_child;
    Node* right_child;
    T value;
};


#endif // CAMERASTABILIZER_TREE_H_COMPONENTTREEPARSER_HPP_INCLUDED
