#ifndef EUROC_STAGE2_COMMON_CSV_PARSER_H
#define EUROC_STAGE2_COMMON_CSV_PARSER_H

#include <Eigen/Eigen>

#include <fstream>
#include <vector>

namespace euroc_stage2 {

// Parses the data into an std::vector of Eigen vectors, of a size set by
// num columns. Attempts to parse all fields as doubles. Can handle both
// comma- and space-separated files.
bool parseCsvIntoVector(const std::string& filename, size_t num_columns,
                        std::vector<Eigen::VectorXd>* output);

}  // namespace euroc_stage2

#endif  // EUROC_STAGE2_COMMON_CSV_PARSER_H
