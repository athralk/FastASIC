#pragma once
#include "gate_forest.hpp"

// Rebuilds every chain of one associative gate type (AND, OR, XOR) as a depth-balanced tree:
// the operands are combined two at a time, always the two shallowest first (Huffman-style), so
// a 22-input XOR chain written as x0 ^ x1 ^ ... becomes 5 levels instead of 21. Gate count is
// unchanged; only gates read by nothing else are merged into a chain. (ABC's `balance` does
// the same on AIGs.)
GateForest balanceAssociative(const GateForest& forest);
