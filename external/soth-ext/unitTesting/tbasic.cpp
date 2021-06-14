/* -------------------------------------------------------------------------- *
 *
 * Unittest of the BasicStage class.
 *
 * -------------------------------------------------------------------------- */

//#define SOTH_DEBUG
//#define SOTH_DEBUG_MODE 45
#include "soth/BaseY.hpp"
#include "soth/BasicStage.hpp"
#include "soth/debug.hpp"

using namespace soth;
using std::endl;

/* -------------------------------------------------------------------------- */

void testBasicStage() {
  MatrixXd m1(5, 4);
  Map<MatrixXd> map1(m1.data(), m1.size(), 1);
  map1 = VectorXd::LinSpaced((long int)m1.size(), 0.0, (double)m1.size() - 1);
  std::cout << "m1 = " << m1 << endl;

  VectorBound b1(5);
  b1[0] = 1.6;
  b1[1] = std::make_pair(-0.1, 0.2);
  std::cout << "b1 = " << b1 << endl;

  soth::BaseY Y(5);
  soth::BasicStage st(5, 4, m1.data(), b1.data(), Y);
  // st.set( m1.data(),b1.data() );
  st.set(m1, b1);

  std::cout << "J=" << st.getJ() << endl;
  std::cout << "b=" << st.getBounds() << endl;
}

int main(int, char**) { testBasicStage(); }
