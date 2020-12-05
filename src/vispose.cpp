#include <vector>
#include <algorithm>
#include <iostream>
#include <iterator>

struct item_t {
    int x;
    int y;
    item_t( int h, int w ) : x(h), y(w) {}
    friend std::ostream& operator<<(std::ostream& os, const item_t& gt) {
        os << "(" << gt.x << "," << gt.y << ")";
        return os;
    }
};
typedef std::vector<item_t> item_list_t;
typedef item_list_t::iterator item_list_itr_t;

struct compare_xy {
    bool operator ()(const item_t& left, const item_t& right) const {
    return (left.x == right.x ? left.y < right.y : left.x < right.x);
    }
};

int main(int argc, char const *argv[])
{
// Initalizing the vector v with these values
    std::vector<int> item_t{ 1, 5, 8, 9, 6, 7, 3, 4, 2, 0 }; 

// Vector is sorted in ascending order   
    std::sort(item_t.begin(), item_t.end()); 

      for (int i = 0; i < item_t.size(); i++) {
        
        std::cout << item_t.at(i) << ' ';
      
      }

  return 0;
}

