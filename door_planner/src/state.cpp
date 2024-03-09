#include "state.h"
using namespace std;
State::State() : idx(-1), g(-1), h(-1)
{
};
State::State(int idx, double g, double h, double epsilon) : idx(idx), g(g), h(h), epsilon(epsilon)
{
};
