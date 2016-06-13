/*
 * random_vec.hpp
 *
 *  Created on: Mar 14, 2012
 *      Author: acmarkus
 */

#ifndef RANDOM_VEC_HPP_
#define RANDOM_VEC_HPP_

#include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

namespace random_vector {

template <int N, class T = double, class D = boost::uniform_real<T> >
class Distribution {
 public:
  typedef D Type;
  typedef std::vector<Type> TypeVector;

 protected:
  TypeVector distributions;

 public:
  Distribution() : distributions(N){};

  inline const D& operator[](int idx) const { return distributions[idx]; }
};

template <int N, class T = double, class D = boost::uniform_real<T>,
          class GeneratorBaseType = boost::mt19937>
class RandomVector {
 public:
  typedef Distribution<N, T, D> DistType;
  typedef GeneratorBaseType GeneratorBase;
  typedef boost::variate_generator<GeneratorBase&, D>
      Generator;  //< keep the reference to GeneratorBase, otherwise we won't
  // have random numbers
  typedef Eigen::Matrix<double, N, 1> Vector;

 private:
  GeneratorBase gen_base_;
  DistType dist_;

 public:
  RandomVector() {}

  RandomVector(const unsigned int seed) : gen_base_(seed) {}

  RandomVector(const DistType& dist, const unsigned int seed = 12345)
      : gen_base_(seed), dist_(dist) {}

  inline void setDistribution(const DistType& dist) { dist_ = dist; }

  DistType getDist() const { return dist_; }

  Vector operator()() {
    Vector ret;
    for (int i = 0; i < N; i++) {
      Generator gen(gen_base_, dist_[i]);
      ret[i] = gen();
    }
    return ret;
  }

  Vector operator()(const DistType& dist) {
    Vector ret;
    for (int i = 0; i < N; i++) {
      Generator gen(gen_base_, dist[i]);
      ret[i] = gen();
    }
    return ret;
  }

  inline int getDimension() { return N; }
};

template <int N, class T = double>
class NormalDistribution
    : public Distribution<N, T, boost::normal_distribution<T> > {
 public:
  typedef boost::normal_distribution<T> D;
  typedef Distribution<N, T, boost::normal_distribution<T> > Base;

  //    NormalDistribution(const Eigen::Matrix<T, N, 1> & mean, const
  //    Eigen::Matrix<T, N, 1> & sigma)
  //    {
  //      for (int i = 0; i < N; i++)
  //      {
  //        Base::distributions[i] = D(mean(i), sigma(i));
  //      }
  //    }

  template <class DerivedMean, class DerivedSigma>
  NormalDistribution(const Eigen::MatrixBase<DerivedMean>& mean,
                     const Eigen::MatrixBase<DerivedSigma>& sigma) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedMean, N);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedSigma, N);
    for (int i = 0; i < N; i++) {
      Base::distributions[i] = D(mean(i), sigma(i));
    }
  }

  NormalDistribution(const T& mean = 0, const T& sigma = 1)
      : Base::distributions(N, D(mean, sigma)) {}
};

template <int N, class T = double>
class UniformDistribution : public Distribution<N, T, boost::uniform_real<T> > {
 public:
  typedef boost::uniform_real<T> D;
  typedef Distribution<N, T, D> Base;

  UniformDistribution(const Eigen::Matrix<T, N, 1>& min,
                      const Eigen::Matrix<T, N, 1>& max) {
    for (int i = 0; i < N; i++) Base::distributions[i] = D(min(i), max(i));
  }

  UniformDistribution(const T& min = 0, const T& max = 1)
      : Base::distributions(N, D(min, max)) {}
};

}  // end namespace

#endif /* RANDOM_VEC_HPP_ */
