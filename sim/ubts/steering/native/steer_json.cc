// Minimal strict JSON reader (see steer_json.h).
#include "steer_json.h"

#include <stdio.h>
#include <stdlib.h>

#include <set>

namespace ubts_steer {
namespace json {

namespace {

const int kMaxDepth = 64;

bool is_digit(char c) { return c >= '0' && c <= '9'; }

bool is_finite(double x) { return x == x && x <= 1.7976931348623157e308 && x >= -1.7976931348623157e308; }

void append_utf8(std::string *out, unsigned cp) {
  if (cp < 0x80) {
    out->push_back(static_cast<char>(cp));
  } else if (cp < 0x800) {
    out->push_back(static_cast<char>(0xc0 | (cp >> 6)));
    out->push_back(static_cast<char>(0x80 | (cp & 0x3f)));
  } else if (cp < 0x10000) {
    out->push_back(static_cast<char>(0xe0 | (cp >> 12)));
    out->push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3f)));
    out->push_back(static_cast<char>(0x80 | (cp & 0x3f)));
  } else {
    out->push_back(static_cast<char>(0xf0 | (cp >> 18)));
    out->push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3f)));
    out->push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3f)));
    out->push_back(static_cast<char>(0x80 | (cp & 0x3f)));
  }
}

class Parser {
 public:
  Parser(const std::string &text, std::vector<Node> *nodes) : s_(text), pos_(0), nodes_(nodes) {}

  int parse_document() {
    for (size_t i = 0; i < s_.size(); ++i) {
      if (static_cast<unsigned char>(s_[i]) >= 0x80) {
        pos_ = i;
        fail("non-ASCII byte");
      }
    }
    skip_ws();
    const int root = parse_value(0);
    skip_ws();
    if (pos_ != s_.size()) fail("trailing characters after the JSON value");
    return root;
  }

 private:
  void fail(const std::string &what) const {
    char where[48];
    snprintf(where, sizeof where, " at byte %lu", static_cast<unsigned long>(pos_));
    throw ParseError(what + where);
  }

  char peek() const { return pos_ < s_.size() ? s_[pos_] : '\0'; }

  void skip_ws() {
    while (pos_ < s_.size()) {
      const char c = s_[pos_];
      if (c != ' ' && c != '\t' && c != '\n' && c != '\r') break;
      ++pos_;
    }
  }

  int new_node(Kind kind) {
    Node node;
    node.kind = kind;
    nodes_->push_back(node);
    return static_cast<int>(nodes_->size()) - 1;
  }

  void expect_word(const char *word) {
    for (const char *p = word; *p; ++p) {
      if (pos_ >= s_.size() || s_[pos_] != *p) fail(std::string("invalid literal, expected ") + word);
      ++pos_;
    }
  }

  int parse_value(int depth) {
    if (depth > kMaxDepth) fail("nesting deeper than 64");
    if (pos_ >= s_.size()) fail("unexpected end of input");
    const char c = s_[pos_];
    if (c == '{') return parse_object(depth);
    if (c == '[') return parse_array(depth);
    if (c == '"') {
      const std::string value = parse_string();
      const int index = new_node(kString);
      (*nodes_)[static_cast<size_t>(index)].text = value;
      return index;
    }
    if (c == 't' || c == 'f') {
      const bool value = c == 't';
      expect_word(value ? "true" : "false");
      const int index = new_node(kBool);
      (*nodes_)[static_cast<size_t>(index)].boolean = value;
      return index;
    }
    if (c == 'n') {
      expect_word("null");
      return new_node(kNull);
    }
    if (c == '-' || is_digit(c)) return parse_number();
    fail("unexpected character");
    return -1;
  }

  int parse_object(int depth) {
    const int self = new_node(kObject);
    ++pos_;  // '{'
    skip_ws();
    if (peek() == '}') {
      ++pos_;
      return self;
    }
    std::set<std::string> seen;
    for (;;) {
      skip_ws();
      if (peek() != '"') fail("object key must be a string");
      const std::string key = parse_string();
      if (!seen.insert(key).second) fail("duplicate key \"" + key + "\"");
      skip_ws();
      if (peek() != ':') fail("expected ':' after an object key");
      ++pos_;
      skip_ws();
      const int child = parse_value(depth + 1);
      (*nodes_)[static_cast<size_t>(self)].keys.push_back(key);
      (*nodes_)[static_cast<size_t>(self)].items.push_back(child);
      skip_ws();
      const char c = peek();
      if (c == ',') {
        ++pos_;
        continue;
      }
      if (c == '}') {
        ++pos_;
        return self;
      }
      fail("expected ',' or '}' in an object");
    }
  }

  int parse_array(int depth) {
    const int self = new_node(kArray);
    ++pos_;  // '['
    skip_ws();
    if (peek() == ']') {
      ++pos_;
      return self;
    }
    for (;;) {
      skip_ws();
      const int child = parse_value(depth + 1);
      (*nodes_)[static_cast<size_t>(self)].items.push_back(child);
      skip_ws();
      const char c = peek();
      if (c == ',') {
        ++pos_;
        continue;
      }
      if (c == ']') {
        ++pos_;
        return self;
      }
      fail("expected ',' or ']' in an array");
    }
  }

  unsigned hex4() {
    if (pos_ + 4 > s_.size()) fail("short \\u escape");
    unsigned value = 0;
    for (int i = 0; i < 4; ++i) {
      const char c = s_[pos_++];
      value <<= 4;
      if (c >= '0' && c <= '9') {
        value |= static_cast<unsigned>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        value |= static_cast<unsigned>(c - 'a' + 10);
      } else if (c >= 'A' && c <= 'F') {
        value |= static_cast<unsigned>(c - 'A' + 10);
      } else {
        fail("bad hex digit in a \\u escape");
      }
    }
    return value;
  }

  std::string parse_string() {
    ++pos_;  // opening quote
    std::string out;
    for (;;) {
      if (pos_ >= s_.size()) fail("unterminated string");
      const unsigned char c = static_cast<unsigned char>(s_[pos_++]);
      if (c == '"') return out;
      if (c < 0x20) fail("control character in a string");
      if (c != '\\') {
        out.push_back(static_cast<char>(c));
        continue;
      }
      if (pos_ >= s_.size()) fail("unterminated escape");
      const char e = s_[pos_++];
      switch (e) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
          unsigned cp = hex4();
          if (cp >= 0xd800 && cp <= 0xdbff && pos_ + 6 <= s_.size() && s_[pos_] == '\\' && s_[pos_ + 1] == 'u') {
            const size_t save = pos_;
            pos_ += 2;
            const unsigned low = hex4();
            if (low >= 0xdc00 && low <= 0xdfff) {
              cp = 0x10000 + ((cp - 0xd800) << 10) + (low - 0xdc00);
            } else {
              pos_ = save;
            }
          }
          append_utf8(&out, cp);
          break;
        }
        default:
          fail("bad escape in a string");
      }
    }
  }

  int parse_number() {
    const size_t start = pos_;
    bool integer = true;
    if (s_[pos_] == '-') ++pos_;
    if (pos_ >= s_.size()) fail("bad number");
    if (s_[pos_] == '0') {
      ++pos_;
    } else if (is_digit(s_[pos_])) {
      while (pos_ < s_.size() && is_digit(s_[pos_])) ++pos_;
    } else {
      fail("bad number");
    }
    if (pos_ < s_.size() && s_[pos_] == '.') {
      integer = false;
      ++pos_;
      if (pos_ >= s_.size() || !is_digit(s_[pos_])) fail("bad number fraction");
      while (pos_ < s_.size() && is_digit(s_[pos_])) ++pos_;
    }
    if (pos_ < s_.size() && (s_[pos_] == 'e' || s_[pos_] == 'E')) {
      integer = false;
      ++pos_;
      if (pos_ < s_.size() && (s_[pos_] == '+' || s_[pos_] == '-')) ++pos_;
      if (pos_ >= s_.size() || !is_digit(s_[pos_])) fail("bad number exponent");
      while (pos_ < s_.size() && is_digit(s_[pos_])) ++pos_;
    }
    const std::string literal = s_.substr(start, pos_ - start);
    char *end = 0;
    const double value = strtod(literal.c_str(), &end);
    if (end != literal.c_str() + literal.size()) fail("bad number");
    if (!is_finite(value)) fail("number out of range");
    const int index = new_node(kNumber);
    Node &node = (*nodes_)[static_cast<size_t>(index)];
    node.number = value;
    node.integer_literal = integer;
    node.text = literal;
    return index;
  }

  const std::string &s_;
  size_t pos_;
  std::vector<Node> *nodes_;
};

}  // namespace

Document::Document(const std::string &text) : root_(0) {
  nodes_.reserve(256);
  Parser parser(text, &nodes_);
  root_ = parser.parse_document();
}

bool to_uint64(const Node &node, uint64_t *out) {
  if (node.kind != kNumber || !node.integer_literal) return false;
  const std::string &t = node.text;
  if (t == "-0") {
    *out = 0;
    return true;
  }
  if (t.empty() || t[0] == '-') return false;
  uint64_t value = 0;
  for (size_t i = 0; i < t.size(); ++i) {
    const uint64_t digit = static_cast<uint64_t>(t[i] - '0');
    if (value > (0xffffffffffffffffULL - digit) / 10ULL) return false;
    value = value * 10ULL + digit;
  }
  *out = value;
  return true;
}

bool to_int64(const Node &node, int64_t *out) {
  if (node.kind != kNumber || !node.integer_literal) return false;
  const std::string &t = node.text;
  const bool negative = !t.empty() && t[0] == '-';
  uint64_t magnitude = 0;
  for (size_t i = negative ? 1 : 0; i < t.size(); ++i) {
    const uint64_t digit = static_cast<uint64_t>(t[i] - '0');
    if (magnitude > (0x8000000000000000ULL - digit) / 10ULL) return false;
    magnitude = magnitude * 10ULL + digit;
  }
  if (!negative && magnitude > 0x7fffffffffffffffULL) return false;
  *out = negative ? static_cast<int64_t>(0 - magnitude) : static_cast<int64_t>(magnitude);
  return true;
}

}  // namespace json
}  // namespace ubts_steer
