#include "state.h"
using namespace std;
State::State() : idx(-1), g(-1), h(-1)
{
};
State::State(unsigned int idx, double g, double h, double epsilon, unsigned int a) : idx(idx), g(g), h(h), epsilon(epsilon), a(a)
{
};
