// Minimal strict JSON reader for the lockstep startup line (protocol section 2).
//
// Accepts exactly RFC 8259 JSON: objects, arrays, strings (with escapes), numbers, true,
// false, null. Rejects duplicate object keys, non-ASCII bytes, control characters in
// strings, NaN/Infinity, leading zeros, trailing data and nesting deeper than 64.
// Nodes live in one vector and refer to each other by index.
#ifndef UBTS_STEER_JSON_H
#define UBTS_STEER_JSON_H

#include <stdint.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace ubts_steer {
namespace json {

enum Kind { kNull, kBool, kNumber, kString, kArray, kObject };

struct Node {
  Node() : kind(kNull), boolean(false), integer_literal(false), number(0.0) {}
  Kind kind;
  bool boolean;
  bool integer_literal;           // number written without fraction and exponent
  double number;                  // value of a number (strtod of the literal)
  std::string text;               // decoded string, or the literal of a number
  std::vector<int> items;         // array items, or object member values (node indices)
  std::vector<std::string> keys;  // object member keys, in the same order as items
};

class ParseError : public std::runtime_error {
 public:
  explicit ParseError(const std::string &what) : std::runtime_error(what) {}
};

class Document {
 public:
  explicit Document(const std::string &text);  // throws ParseError
  int root() const { return root_; }
  const Node &node(int index) const { return nodes_[static_cast<size_t>(index)]; }

 private:
  std::vector<Node> nodes_;
  int root_;
};

// Integer value of an integer literal; false if the node is not an integer literal or out of range.
bool to_int64(const Node &node, int64_t *out);
bool to_uint64(const Node &node, uint64_t *out);

}  // namespace json
}  // namespace ubts_steer

#endif  // UBTS_STEER_JSON_H
