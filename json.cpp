#include "json.h"
#include <cmath>
using namespace std;

namespace Json {

  Document::Document(Node root) : root(move(root)) {
  }

  const Node& Document::GetRoot() const {
    return root;
  }

  Node LoadNode(istream& input);

  Node LoadArray(istream& input) {
    vector<Node> result;

    for (char c; input >> c && c != ']'; ) {
      if (c != ',') {
        input.putback(c);
      }
      result.push_back(LoadNode(input));
    }

    return Node(move(result));
  }

  Node LoadDouble(istream& input, double res, bool flag_negative) {
	  double r = res;
	  uint64_t tmp = 0;
	  int64_t counter = 0;
	  input.ignore(1);
	  while (isdigit(input.peek())) {
		  tmp *= 10;
		  tmp += input.get() - '0';
		  counter--;
	  }
	  double p = static_cast<double>(tmp) * pow(10, counter);
	  flag_negative ? r -= p : r += p;
	  return Node(r);
  }

  Node LoadBool(istream& input) {
	  char first_letter = input.peek();
	  bool ret;
	  (first_letter == 't') ? ret = true : ret = false;
	  (first_letter == 't') ? input.ignore(4) : input.ignore(5);
	  return Node(ret);
  }

  Node LoadInt(istream& input) {
    int result = 0;
	bool flag_neg = false;
    while (isdigit(input.peek())) {
      result *= 10;
      result += input.get() - '0';
    }
	char c = input.peek();
	if (c == '.') {
		return LoadDouble(input, static_cast<double>(result), flag_neg);
	}
	else if (c == '-') {
		flag_neg = true;
		input.ignore(1);
		while (isdigit(input.peek())) {
			result *= 10;
			result += input.get() - '0';
		}
		result *= -1;
		char c = input.peek();
		if (c == '.') {
			return LoadDouble(input, static_cast<double>(result), flag_neg);
		}
		return Node(result);
	}
    return Node(result);
  }

  Node LoadString(istream& input) {
    string line;
    getline(input, line, '"');
    return Node(move(line));
  }

  Node LoadDict(istream& input) {
    map<string, Node> result;

    for (char c; input >> c && c != '}'; ) {
      if (c == ',') {
        input >> c;
      }

      string key = LoadString(input).AsString();
      input >> c;
      result.emplace(move(key), LoadNode(input));
    }

    return Node(move(result));
  }

  Node LoadNode(istream& input) {
    char c;
    input >> c;

    if (c == '[') {
      return LoadArray(input);
    } else if (c == '{') {
      return LoadDict(input);
    } else if (c == '"') {
      return LoadString(input);
	}
	else if (c == 'f' || c == 't') {
		input.putback(c);
		return LoadBool(input);
	}
	else {
      input.putback(c);
      return LoadInt(input);
    }
  }

  Document Load(istream& input) {
    return Document{LoadNode(input)};
  }

}
