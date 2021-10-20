/* -------------------------------------------------------------------------- *
 *
 * Generation of random problems.
 *
 * -------------------------------------------------------------------------- */

#ifndef __SOTH_RANDOM_GENERATOR__
#define __SOTH_RANDOM_GENERATOR__

#include <vector>
#include "../include/soth/Algebra.hpp"
#include "../include/soth/Bound.hpp"

namespace soth {
/* Being specified the size of the problem NC, the number of stage NB_STAGE,
 * the size of each size NR, the rank of the stage wrt. the previous stages
 * RANKFREE = rank(Ji.Pi-1), and the rank due to the previous stage
 * RANKLINKED = rank(Ji) - RANKFREE, the function generates in J and b
 * a random problems with good conditionning numbers (ie =0 XOR >>0).
 */
void generateDeficientDataSet(std::vector<Eigen::MatrixXd> &J,
                              std::vector<soth::VectorBound> &b,
                              const unsigned int NB_STAGE,
                              const std::vector<unsigned int> &RANKFREE,
                              const std::vector<unsigned int> &RANKLINKED,
                              const std::vector<unsigned int> &NR,
                              const unsigned int NC);

void generateDeficientDataSet(std::vector<Eigen::MatrixXd> &J,
                              std::vector<soth::VectorBound> &b,
                              const unsigned int NB_STAGE, const int RANKFREE[],
                              const int RANKLINKED[], const int NR[],
                              const unsigned int NC);

/* Generated randomly a profile of problem, ie sizes and ranks. The ouput
 * of this functions are to be sent to the previous function. */
void generateRandomProfile(unsigned int &nbStage,
                           std::vector<unsigned int> &rankfree,
                           std::vector<unsigned int> &ranklinked,
                           std::vector<unsigned int> &nr, unsigned int &nc);

/* Input: size = number of cols, rowPercent = row number / col number,
 * rankPercent = rank / col number,
 * selfDeficiencyPercent = rankdef [A1 .. Ap] / sum rankdef(Ai)
 */
void generateFixedSizeRandomProfile(
    const unsigned int size, const double rowPercent, const double rankPercent,
    const double selfDeficiencyPercent, unsigned int &nbStage,
    std::vector<unsigned int> &rankfree, std::vector<unsigned int> &ranklinked,
    std::vector<unsigned int> &nr, unsigned int &nc);

void randomProblem(std::vector<Eigen::MatrixXd> &J,
                   std::vector<soth::VectorBound> &b);

/* --- IN/OUT PROBLEMS --- */
void readProblemFromFile(const std::string name,
                         std::vector<Eigen::MatrixXd> &J,
                         std::vector<soth::VectorBound> &b,
                         unsigned int &NB_STAGE, std::vector<unsigned int> &NR,
                         unsigned int &NC);
void readProblemFromFile(const std::string name,
                         std::vector<Eigen::MatrixXd> &J,
                         std::vector<soth::VectorBound> &b);
void readProblemFromBinFile(const std::string name,
                            std::vector<Eigen::MatrixXd> &J,
                            std::vector<soth::VectorBound> &b,
                            unsigned int &NB_STAGE,
                            std::vector<unsigned int> &NR, unsigned int &NC);

void writeProblemToFile(const std::string name,
                        const std::vector<Eigen::MatrixXd> &J,
                        const std::vector<soth::VectorBound> &b,
                        const unsigned int &NB_STAGE,
                        const std::vector<unsigned int> &NR,
                        const unsigned int &NC);

void writeProblemToFile(const std::string name,
                        const std::vector<Eigen::MatrixXd> &J,
                        const std::vector<soth::VectorBound> &b);
}  // namespace soth

#endif  // #ifndef __SOTH_RANDOM_GENERATOR__
