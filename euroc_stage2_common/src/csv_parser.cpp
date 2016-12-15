#include <iostream>

#include <euroc_stage2/csv_parser.h>

namespace euroc_stage2 {

// Parses the data into an std::vector of Eigen vectors, of a size set by
// num columns. Attempts to parse all fields as doubles. Can handle both
// comma- and space-separated files.
bool parseCsvIntoVector(const std::string& filename, size_t num_columns,
                        std::vector<Eigen::VectorXd>* output) {
  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
    return false;
  }

  output->clear();

  Eigen::VectorXd column(num_columns);
  column.setZero();
  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);
    // Check how this line is separated.
    // Check if this starts with an invalid character: # or [
    // Then ignore and just get the next line.
    if (line_stream.peek() == '#' || line_stream.peek() == '[') {
      continue;
    }

    const bool comma_separated = (line.find(',') != std::string::npos);
    if (line_stream.eof()) {
      return false;
    }

    for (size_t i = 0; i < num_columns; ++i) {
      if (line_stream.eof()) {
        return false;
      }
      std::string element;
      std::getline(line_stream, element, (comma_separated ? ',' : ' '));
      try {
        column(i) = std::stod(element);
      } catch (const std::exception& exception) {
        std::cout << "Could not parse number in import file.\n";
        return false;
      }
    }
    output->push_back(column);
  }

  return true;
}

}  // namespace euroc_stage2
