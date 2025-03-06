#pragma once
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cfloat>
#include <vector>
#include <eigen3/Eigen/Eigen>

#include "utils/root_finder.hpp"
#include "utils/banded_system.hpp"

namespace trailer_planner
{
  // row vector with value: c0, c1, c2, ...
  template<int D, int Order>
  using CoefficientMat = Eigen::Matrix<double, D, Order+1>;

  template<int D, int Order>
  using VelCoefficientMat = Eigen::Matrix<double, D, Order>;

  template<int D, int Order>
  using AccCoefficientMat = Eigen::Matrix<double, D, Order-1>;

  template<int D, int Order>
  using JerCoefficientMat = Eigen::Matrix<double, D, Order-2>;
  
  template <int Dim, int Order>
  class Piece
  {
  private:
      double duration;
      CoefficientMat<Dim, Order> coeffMat;
      int order = Order;
      int direction;

  public:
      Piece() = default;

      Piece(double dur, const CoefficientMat<Dim, Order> &cMat)
          : duration(dur), coeffMat(cMat), direction(1) {}

      Piece(double dur, const CoefficientMat<Dim, Order> &cMat, int direct)
          : duration(dur), coeffMat(cMat), direction(direct) {}
      
      inline int getDim() const
      {
          return Dim;
      }
      
      inline int getOrder() const
      {
          return order;
      }

      inline double getDuration() const
      {
          return duration;
      }

      inline int getDirection() const
      {
          return direction;
      }

      inline const CoefficientMat<Dim, Order> &getCoeffMat() const
      {
          return coeffMat;
      }

      inline CoefficientMat<Dim, Order> normalizedCoeffMat() const
      {
          CoefficientMat<Dim, Order> nCoeffsMat;
          double t = 1.0;
          for (int i = order; i >= 0; i--)
          {
              nCoeffsMat.col(i) = coeffMat.col(i) * t;
              t *= duration;
          }
          return nCoeffsMat;
      }

      inline VelCoefficientMat<Dim, Order> normalizedVelCoeffMat() const
      {
          VelCoefficientMat<Dim, Order> nDotCoeffsMat;
          int n = 1;
          double t = duration;
          for (int i = order - 1; i >= 0; i--)
          {
              nDotCoeffsMat.col(i) = n * coeffMat.col(i) * t;
              t *= duration;
              n++;
          }
          return nDotCoeffsMat;
      }

      inline AccCoefficientMat<Dim, Order> normalizedAccCoeffMat() const
      {
          AccCoefficientMat<Dim, Order> nDDotCoeffsMat;
          int n = 2;
          int m = 1;
          double t = duration * duration;
          for (int i = order - 2; i >= 0; i--)
          {
              nDDotCoeffsMat.col(i) = n * m * coeffMat.col(i) * t;
              n++;
              m++;
              t *= duration;
          }
          return nDDotCoeffsMat;
      }

      inline Eigen::Matrix<double, Dim, 1> getPos(const double &t) const
      {
          Eigen::Matrix<double, Dim, 1> value = Eigen::Matrix<double, Dim, 1>::Zero();
          double tn = 1.0;

          for (int i = order; i >= 0; i--)
          {
              value += tn * coeffMat.col(i);
              tn *= t;
          }

          return value;
      }

      inline Eigen::Matrix<double, Dim, 1> getVel(const double &t) const
      {
          Eigen::Matrix<double, Dim, 1> dvalue = Eigen::Matrix<double, Dim, 1>::Zero();
          double tn = 1.0;
          int n = 1;

          for (int i = order-1; i >= 0; i--)
          {
              dvalue += n * tn * coeffMat.col(i);
              tn *= t;
              n++;
          }

          return dvalue;
      }

      inline Eigen::Matrix<double, Dim, 1> getAcc(const double &t) const
      {
          Eigen::Matrix<double, Dim, 1> ddvalue = Eigen::Matrix<double, Dim, 1>::Zero();
          double tn = 1.0;
          int m = 1;
          int n = 2;
          for (int i = order-2; i >= 0; i--)
          {
              ddvalue += m * n * tn * coeffMat.col(i);
              tn *= t;
              m++;
              n++;
          }
          return ddvalue;
      }

      inline Eigen::Matrix<double, Dim, 1> getJer(const double &t) const
      {
          Eigen::Matrix<double, Dim, 1> dddvalue = Eigen::Matrix<double, Dim, 1>::Zero();
          double tn = 1.0;
          int m = 1;
          int n = 2;
          for (int i = order-3; i >= 0; i--)
          {
              dddvalue += m * n * tn * coeffMat.col(i);
              tn *= t;
              m++;
              n++;
          }
          return dddvalue;
      }

      inline double getMaxVelNorm() const
      {
          VelCoefficientMat<Dim, Order> nDsigmaCoeffMat = normalizedVelCoeffMat();
          Eigen::VectorXd coeff = RootFinder::polySqr(nDsigmaCoeffMat.row(0));
          for(int i = 1; i < Dim; i++)
          {
              coeff = coeff + RootFinder::polySqr(nDsigmaCoeffMat.row(i));
          }
          int N = coeff.size();
          int n = N - 1;
          for (int i = 0; i < N; i++)
          {
              coeff(i) *= n;
              n--;
          }
          if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
          {
              return getVel(0.0).norm();
          }
          else
          {
              double l = -0.0625;
              double r = 1.0625;
              while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
              {
                  l = 0.5 * l;
              }
              while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
              {
                  r = 0.5 * (r + 1.0);
              }
              std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                      FLT_EPSILON / duration);
              candidates.insert(0.0);
              candidates.insert(1.0);
              double maxVelRateSqr = -INFINITY;
              double tempNormSqr;
              for (std::set<double>::const_iterator it = candidates.begin();
                  it != candidates.end();
                  it++)
              {
                  if (0.0 <= *it && 1.0 >= *it)
                  {
                      tempNormSqr = getVel((*it) * duration).norm() * getVel((*it) * duration).norm();
                      maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
                  }
              }
              return sqrt(maxVelRateSqr);
          }
      }

      inline double getMaxAccNorm() const
      {
          AccCoefficientMat<Dim, Order> nDDotCoeffsMat = normalizedAccCoeffMat();
          Eigen::VectorXd coeff = RootFinder::polySqr(nDDotCoeffsMat.row(0));
          for(int i = 1; i < Dim; i++){
              coeff = coeff + RootFinder::polySqr(nDDotCoeffsMat.row(i));
          }
          int N = coeff.size();
          int n = N - 1;
          for (int i = 0; i < N; i++)
          {
              coeff(i) *= n;
              n--;
          }
          if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
          {
              return getAcc(0.0).norm();
          }
          else
          {
              double l = -0.0625;
              double r = 1.0625;
              while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
              {
                  l = 0.5 * l;
              }
              while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
              {
                  r = 0.5 * (r + 1.0);
              }
              std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                      FLT_EPSILON / duration);
              candidates.insert(0.0);
              candidates.insert(1.0);
              double maxAccRateSqr = -INFINITY;
              double tempNormSqr;
              for (std::set<double>::const_iterator it = candidates.begin();
                  it != candidates.end();
                  it++)
              {
                  if (0.0 <= *it && 1.0 >= *it)
                  {
                      tempNormSqr = getAcc((*it) * duration).norm() * getAcc((*it) * duration).norm();
                      maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
                  }
              }
              return sqrt(maxAccRateSqr);
          }
      }
  };

  template <int Dim, int Order>
  class PolyTrajectory
  {
  private:
      using Pieces = std::vector<Piece<Dim, Order>>;
      Pieces pieces;

  public:
      PolyTrajectory() = default;

      PolyTrajectory(const std::vector<double> &durs,
                    const std::vector<CoefficientMat<Dim, Order>> &cMats)
      {
          int N = std::min(durs.size(), cMats.size());
          pieces.reserve(N);
          for (int i = 0; i < N; i++)
          {
              pieces.emplace_back(durs[i], cMats[i], 1);
          }
      }

      PolyTrajectory(const std::vector<double> &durs,
                    const std::vector<CoefficientMat<Dim, Order>> &cMats,
                    const std::vector<int> directs)
      {
          int N = std::min(durs.size(), cMats.size(), directs.size());
          pieces.reserve(N);
          for (int i = 0; i < N; i++)
          {
              pieces.emplace_back(durs[i], cMats[i], directs[i]);
          }
      }

      inline int getPieceNum() const
      {
          return pieces.size();
      }

      inline Eigen::VectorXd getDurations() const
      {
          int N = getPieceNum();
          Eigen::VectorXd durations(N);
          for (int i = 0; i < N; i++)
          {
              durations(i) = pieces[i].getDuration();
          }
          return durations;
      }

      inline Eigen::VectorXi getDirections() const
      {
          int N = getPieceNum();
          Eigen::VectorXi directions(N);
          for (int i = 0; i < N; i++)
          {
              directions(i) = pieces[i].getDirection();
          }
          return directions;
      }

      inline double getTotalDuration() const
      {
          int N = getPieceNum();
          double totalDuration = 0.0;
          for (int i = 0; i < N; i++)
          {
              totalDuration += pieces[i].getDuration();
          }
          return totalDuration;
      }

      inline const Piece<Dim, Order> &operator[](int i) const
      {
          return pieces[i];
      }

      inline Piece<Dim, Order> &operator[](int i)
      {
          return pieces[i];
      }

      inline void clear(void)
      {
          pieces.clear();
          return;
      }

      inline void reserve(const int &n)
      {
          pieces.reserve(n);
          return;
      }

      inline void emplace_back(const Piece<Dim, Order> &piece)
      {
          pieces.emplace_back(piece);
          return;
      }

      inline void emplace_back(const double &dur,
                              const CoefficientMat<Dim, Order> &cMat)
      {
          pieces.emplace_back(dur, cMat);
          return;
      }

      inline void emplace_back(const double &dur,
                              const CoefficientMat<Dim, Order> &cMat,
                              const std::vector<int> directs)
      {
          pieces.emplace_back(dur, cMat, directs);
          return;
      }

      inline void append(const PolyTrajectory<Dim, Order> &traj)
      {
          pieces.insert(pieces.end(), traj.begin(), traj.end());
          return;
      }

      inline int locatePieceIdx(double &t) const
      {
          int N = getPieceNum();
          int idx;
          double dur;
          for (idx = 0;
              idx < N &&
              t > (dur = pieces[idx].getDuration());
              idx++)
          {
              t -= dur;
          }
          if (idx == N)
          {
              idx--;
              t += pieces[idx].getDuration();
          }
          return idx;
      }

      inline int getDirection(double t) const
      {
          int pieceIdx = locatePieceIdx(t);
          return pieces[pieceIdx].getDirection();
      }

      inline Eigen::Matrix<double, Dim, 1> getPos(double t) const
      {
          int pieceIdx = locatePieceIdx(t);
          return pieces[pieceIdx].getPos(t);
      }

      inline Eigen::Matrix<double, Dim, 1> getVel(double t) const
      {
          int pieceIdx = locatePieceIdx(t);
          return pieces[pieceIdx].getVel(t);
      }

      inline Eigen::Matrix<double, Dim, 1> getAcc(double t) const
      {
          int pieceIdx = locatePieceIdx(t);
          return pieces[pieceIdx].getAcc(t);
      }

      inline Eigen::Matrix<double, Dim, 1> getJer(double t) const
      {
          int pieceIdx = locatePieceIdx(t);
          return pieces[pieceIdx].getJer(t);
      }

      inline double getMaxVelNorm() const
      {
          int N = getPieceNum();
          double maxDotValueNorm = -INFINITY;
          double tempNorm;
          for (int i = 0; i < N; i++)
          {
              tempNorm = pieces[i].getMaxVelNorm();
              maxDotValueNorm = maxDotValueNorm < tempNorm ? tempNorm : maxDotValueNorm;
          }
          return maxDotValueNorm;
      }

      inline double getMaxAccNorm() const
      {
          int N = getPieceNum();
          double maxDDotValueNorm = -INFINITY;
          double tempNorm;
          for (int i = 0; i < N; i++)
          {
              tempNorm = pieces[i].getMaxAccNorm();
              maxDDotValueNorm = maxDDotValueNorm < tempNorm ? tempNorm : maxDDotValueNorm;
          }
          return maxDDotValueNorm;
      }
  };

  template <int Dim>  class MinSnapOpt
  {
  public:
    int N;
    Eigen::VectorXd T1;
    BandedSystem A;
    Eigen::MatrixXd b;

    // Temp variables
    Eigen::VectorXd T2;
    Eigen::VectorXd T3;
    Eigen::VectorXd T4;
    Eigen::VectorXd T5;
    Eigen::VectorXd T6;
    Eigen::VectorXd T7;

    MinSnapOpt() = default;
    ~MinSnapOpt() { A.destroy(); }

    inline void reset(const int &pieceNum) 
    {
      N = pieceNum;
      T1.resize(N);
      A.create(8 * N, 8, 8);
      b.resize(8 * N, Dim);
      T2.resize(N);
      T3.resize(N);
      T4.resize(N);
      T5.resize(N);
      T6.resize(N);
      T7.resize(N);
      return;
    }

    // from q,T to c,T
    inline void generate(const Eigen::MatrixXd &headPVAJ,
                        const Eigen::MatrixXd &tailPVAJ,
                        const Eigen::MatrixXd &inPs,
                        const Eigen::VectorXd &ts) 
    {
      T1 = ts;
      T2 = T1.cwiseProduct(T1);
      T3 = T2.cwiseProduct(T1);
      T4 = T2.cwiseProduct(T2);
      T5 = T4.cwiseProduct(T1);
      T6 = T4.cwiseProduct(T2);
      T7 = T4.cwiseProduct(T3);

      A.reset();
      b.setZero();

      A(0, 0) = 1.0;
      A(1, 1) = 1.0;
      A(2, 2) = 2.0;
      A(3, 3) = 6.0;
      b.row(0) = headPVAJ.col(0).transpose();
      b.row(1) = headPVAJ.col(1).transpose();
      b.row(2) = headPVAJ.col(2).transpose();
      b.row(3) = headPVAJ.col(3).transpose();

      for (int i = 0; i < N - 1; i++) 
      {
        A(8 * i + 4, 8 * i + 4) = 24.0;
        A(8 * i + 4, 8 * i + 5) = 120.0 * T1(i);
        A(8 * i + 4, 8 * i + 6) = 360.0 * T2(i);
        A(8 * i + 4, 8 * i + 7) = 840.0 * T3(i);
        A(8 * i + 4, 8 * i + 12) = -24.0;

        A(8 * i + 5, 8 * i + 5) = 120.0;
        A(8 * i + 5, 8 * i + 6) = 720.0 * T1(i);
        A(8 * i + 5, 8 * i + 7) = 2520.0 * T2(i);
        A(8 * i + 5, 8 * i + 13) = -120.0;

        A(8 * i + 6, 8 * i + 6) = 720.0;
        A(8 * i + 6, 8 * i + 7) = 5040.0 * T1(i);
        A(8 * i + 6, 8 * i + 14) = -720.0;

        A(8 * i + 7, 8 * i) = 1.0;
        A(8 * i + 7, 8 * i + 1) = T1(i);
        A(8 * i + 7, 8 * i + 2) = T2(i);
        A(8 * i + 7, 8 * i + 3) = T3(i);
        A(8 * i + 7, 8 * i + 4) = T4(i);
        A(8 * i + 7, 8 * i + 5) = T5(i);
        A(8 * i + 7, 8 * i + 6) = T6(i);
        A(8 * i + 7, 8 * i + 7) = T7(i);

        A(8 * i + 8, 8 * i) = 1.0;
        A(8 * i + 8, 8 * i + 1) = T1(i);
        A(8 * i + 8, 8 * i + 2) = T2(i);
        A(8 * i + 8, 8 * i + 3) = T3(i);
        A(8 * i + 8, 8 * i + 4) = T4(i);
        A(8 * i + 8, 8 * i + 5) = T5(i);
        A(8 * i + 8, 8 * i + 6) = T6(i);
        A(8 * i + 8, 8 * i + 7) = T7(i);
        A(8 * i + 8, 8 * i + 8) = -1.0;

        A(8 * i + 9, 8 * i + 1) = 1.0;
        A(8 * i + 9, 8 * i + 2) = 2.0 * T1(i);
        A(8 * i + 9, 8 * i + 3) = 3.0 * T2(i);
        A(8 * i + 9, 8 * i + 4) = 4.0 * T3(i);
        A(8 * i + 9, 8 * i + 5) = 5.0 * T4(i);
        A(8 * i + 9, 8 * i + 6) = 6.0 * T5(i);
        A(8 * i + 9, 8 * i + 7) = 7.0 * T6(i);
        A(8 * i + 9, 8 * i + 9) = -1.0;

        A(8 * i + 10, 8 * i + 2) = 2.0;
        A(8 * i + 10, 8 * i + 3) = 6.0 * T1(i);
        A(8 * i + 10, 8 * i + 4) = 12.0 * T2(i);
        A(8 * i + 10, 8 * i + 5) = 20.0 * T3(i);
        A(8 * i + 10, 8 * i + 6) = 30.0 * T4(i);
        A(8 * i + 10, 8 * i + 7) = 42.0 * T5(i);
        A(8 * i + 10, 8 * i + 10) = -2.0;

        A(8 * i + 11, 8 * i + 3) = 6.0;
        A(8 * i + 11, 8 * i + 4) = 24.0 * T1(i);
        A(8 * i + 11, 8 * i + 5) = 60.0 * T2(i);
        A(8 * i + 11, 8 * i + 6) = 120.0 * T3(i);
        A(8 * i + 11, 8 * i + 7) = 210.0 * T4(i);
        A(8 * i + 11, 8 * i + 11) = -6.0;

        b.row(8 * i + 7) = inPs.col(i).transpose();
      }

      A(8 * N - 4, 8 * N - 8) = 1.0;
      A(8 * N - 4, 8 * N - 7) = T1(N - 1);
      A(8 * N - 4, 8 * N - 6) = T2(N - 1);
      A(8 * N - 4, 8 * N - 5) = T3(N - 1);
      A(8 * N - 4, 8 * N - 4) = T4(N - 1);
      A(8 * N - 4, 8 * N - 3) = T5(N - 1);
      A(8 * N - 4, 8 * N - 2) = T6(N - 1);
      A(8 * N - 4, 8 * N - 1) = T7(N - 1);

      A(8 * N - 3, 8 * N - 7) = 1.0;
      A(8 * N - 3, 8 * N - 6) = 2.0 * T1(N - 1);
      A(8 * N - 3, 8 * N - 5) = 3.0 * T2(N - 1);
      A(8 * N - 3, 8 * N - 4) = 4.0 * T3(N - 1);
      A(8 * N - 3, 8 * N - 3) = 5.0 * T4(N - 1);
      A(8 * N - 3, 8 * N - 2) = 6.0 * T5(N - 1);
      A(8 * N - 3, 8 * N - 1) = 7.0 * T6(N - 1);

      A(8 * N - 2, 8 * N - 6) = 2.0;
      A(8 * N - 2, 8 * N - 5) = 6.0 * T1(N - 1);
      A(8 * N - 2, 8 * N - 4) = 12.0 * T2(N - 1);
      A(8 * N - 2, 8 * N - 3) = 20.0 * T3(N - 1);
      A(8 * N - 2, 8 * N - 2) = 30.0 * T4(N - 1);
      A(8 * N - 2, 8 * N - 1) = 42.0 * T5(N - 1);

      A(8 * N - 1, 8 * N - 5) = 6.0;
      A(8 * N - 1, 8 * N - 4) = 24.0 * T1(N - 1);
      A(8 * N - 1, 8 * N - 3) = 60.0 * T2(N - 1);
      A(8 * N - 1, 8 * N - 2) = 120.0 * T3(N - 1);
      A(8 * N - 1, 8 * N - 1) = 210.0 * T4(N - 1);

      b.row(8 * N - 4) = tailPVAJ.col(0).transpose();
      b.row(8 * N - 3) = tailPVAJ.col(1).transpose();
      b.row(8 * N - 2) = tailPVAJ.col(2).transpose();
      b.row(8 * N - 1) = tailPVAJ.col(3).transpose();

      A.factorizeLU();
      A.solve(b);

      return;
    }

    inline PolyTrajectory<Dim, 7> getTraj(void) const 
    {
      PolyTrajectory<Dim, 7> traj;
      traj.reserve(N);
      for (int i = 0; i < N; i++) 
      {
        traj.emplace_back(T1(i), b.block<8, Dim>(8 * i, 0).transpose().rowwise().reverse());
      }
      return traj;
    }

    inline double getTrajSnapCost() const
    {
      double energy = 0.0;
      for (int i = 0; i < N; i++)
      {
          energy += 576.0 * b.row(8 * i + 4).squaredNorm() * T1(i) +
                    2880.0 * b.row(8 * i + 4).dot(b.row(8 * i + 5)) * T2(i) +
                    4800.0 * b.row(8 * i + 5).squaredNorm() * T3(i) +
                    5760.0 * b.row(8 * i + 4).dot(b.row(8 * i + 6)) * T3(i) +
                    21600.0 * b.row(8 * i + 5).dot(b.row(8 * i + 6)) * T4(i) +
                    10080.0 * b.row(8 * i + 4).dot(b.row(8 * i + 7)) * T4(i) +
                    25920.0 * b.row(8 * i + 6).squaredNorm() * T5(i) +
                    40320.0 * b.row(8 * i + 5).dot(b.row(8 * i + 7)) * T5(i) +
                    100800.0 * b.row(8 * i + 6).dot(b.row(8 * i + 7)) * T6(i) +
                    100800.0 * b.row(8 * i + 7).squaredNorm() * T7(i);
      }
      return energy;
    }

    inline const Eigen::MatrixXd &getCoeffs(void) const
    {
        return b;
    }

    // know J=∫s²dt
    // then get ∂J/∂c, ∂J/∂T 
    inline void calSnapGradCT(Eigen::MatrixXd& gdC, Eigen::VectorXd &gdT) 
    {
      gdC.resize(8 * N, Dim);
      for (int i = 0; i < N; i++) 
      {
        gdC.row(8 * i + 7) = 10080.0 * b.row(8 * i + 4) * T4(i) +
                             40320.0 * b.row(8 * i + 5) * T5(i) +
                             100800.0 * b.row(8 * i + 6) * T6(i) +
                             201600.0 * b.row(8 * i + 7) * T7(i);
        gdC.row(8 * i + 6) = 5760.0 * b.row(8 * i + 4) * T3(i) +
                             21600.0 * b.row(8 * i + 5) * T4(i) +
                             51840.0 * b.row(8 * i + 6) * T5(i) +
                             100800.0 * b.row(8 * i + 7) * T6(i);
        gdC.row(8 * i + 5) = 2880.0 * b.row(8 * i + 4) * T2(i) +
                             9600.0 * b.row(8 * i + 5) * T3(i) +
                             21600.0 * b.row(8 * i + 6) * T4(i) +
                             40320.0 * b.row(8 * i + 7) * T5(i);
        gdC.row(8 * i + 4) = 1152.0 * b.row(8 * i + 4) * T1(i) +
                             2880.0 * b.row(8 * i + 5) * T2(i) +
                             5760.0 * b.row(8 * i + 6) * T3(i) +
                             10080.0 * b.row(8 * i + 7) * T4(i);
        gdC.block<4, Dim>(8 * i, 0).setZero();
      }

      gdT.resize(N);
      for (int i = 0; i < N; i++) 
      {
        gdT(i) = 576.0 * b.row(8 * i + 4).squaredNorm() +
                 5760.0 * b.row(8 * i + 4).dot(b.row(8 * i + 5)) * T1(i) +
                 14400.0 * b.row(8 * i + 5).squaredNorm() * T2(i) +
                 17280.0 * b.row(8 * i + 4).dot(b.row(8 * i + 6)) * T2(i) +
                 86400.0 * b.row(8 * i + 5).dot(b.row(8 * i + 6)) * T3(i) +
                 40320.0 * b.row(8 * i + 4).dot(b.row(8 * i + 7)) * T3(i) +
                 129600.0 * b.row(8 * i + 6).squaredNorm() * T4(i) +
                 201600.0 * b.row(8 * i + 5).dot(b.row(8 * i + 7)) * T4(i) +
                 604800.0 * b.row(8 * i + 6).dot(b.row(8 * i + 7)) * T5(i) +
                 705600.0 * b.row(8 * i + 7).squaredNorm() * T6(i);
      }
      return;
    }

    // know ∂K/∂C, ∂K/∂T, K(C(q,T),T) = W(q,T)
    // then get ∂W/∂q, ∂W/∂T
    inline void calGradCTtoQT(const Eigen::MatrixXd& gdC, Eigen::VectorXd& gdT, Eigen::MatrixXd& gdP, \
                              Eigen::MatrixXd& gdTail) 
    {
      gdP.resize(Dim, N - 1);
      Eigen::MatrixXd adjGrad = gdC;
      A.solveAdj(adjGrad);

      for (int i = 0; i < N - 1; i++) 
      {
        gdP.col(i) = adjGrad.row(8 * i + 7).transpose();
      }

      gdTail.resize(Dim, 4);
      gdTail = adjGrad.bottomRows(4).transpose();

      Eigen::Matrix<double, 8, Dim> B1;
      Eigen::Matrix<double, 4, Dim> B2;
      for (int i = 0; i < N - 1; i++) 
      {
        // negative velocity
        B1.row(3) = -(b.row(i * 8 + 1) +
                      2.0 * T1(i) * b.row(i * 8 + 2) +
                      3.0 * T2(i) * b.row(i * 8 + 3) +
                      4.0 * T3(i) * b.row(i * 8 + 4) +
                      5.0 * T4(i) * b.row(i * 8 + 5) +
                      6.0 * T5(i) * b.row(i * 8 + 6) +
                      7.0 * T6(i) * b.row(i * 8 + 7));
        B1.row(4) = B1.row(3);

        // negative acceleration
        B1.row(5) = -(2.0 * b.row(i * 8 + 2) +
                      6.0 * T1(i) * b.row(i * 8 + 3) +
                      12.0 * T2(i) * b.row(i * 8 + 4) +
                      20.0 * T3(i) * b.row(i * 8 + 5) +
                      30.0 * T4(i) * b.row(i * 8 + 6) +
                      42.0 * T5(i) * b.row(i * 8 + 7));

        // negative jerk
        B1.row(6) = -(6.0 * b.row(i * 8 + 3) +
                      24.0 * T1(i) * b.row(i * 8 + 4) +
                      60.0 * T2(i) * b.row(i * 8 + 5) +
                      120.0 * T3(i) * b.row(i * 8 + 6) +
                      210.0 * T4(i) * b.row(i * 8 + 7));

        // negative snap
        B1.row(7) = -(24.0 * b.row(i * 8 + 4) +
                      120.0 * T1(i) * b.row(i * 8 + 5) +
                      360.0 * T2(i) * b.row(i * 8 + 6) +
                      840.0 * T3(i) * b.row(i * 8 + 7));

        // negative crackle
        B1.row(0) = -(120.0 * b.row(i * 8 + 5) +
                      720.0 * T1(i) * b.row(i * 8 + 6) +
                      2520.0 * T2(i) * b.row(i * 8 + 7));

        // negative d_crackle
        B1.row(1) = -(720.0 * b.row(i * 8 + 6) +
                      5040.0 * T1(i) * b.row(i * 8 + 7));

        // negative dd_crackle
        B1.row(2) = -5040.0 * b.row(i * 8 + 7);

        gdT(i) += B1.cwiseProduct(adjGrad.block<8, Dim>(8 * i + 4, 0)).sum();
      }

      // negative velocity
      B2.row(0) = -(b.row(8 * N - 7) +
                    2.0 * T1(N - 1) * b.row(8 * N - 6) +
                    3.0 * T2(N - 1) * b.row(8 * N - 5) +
                    4.0 * T3(N - 1) * b.row(8 * N - 4) +
                    5.0 * T4(N - 1) * b.row(8 * N - 3) +
                    6.0 * T5(N - 1) * b.row(8 * N - 2) +
                    7.0 * T6(N - 1) * b.row(8 * N - 1));

      // negative acceleration
      B2.row(1) = -(2.0 * b.row(8 * N - 6) +
                    6.0 * T1(N - 1) * b.row(8 * N - 5) +
                    12.0 * T2(N - 1) * b.row(8 * N - 4) +
                    20.0 * T3(N - 1) * b.row(8 * N - 3) +
                    30.0 * T4(N - 1) * b.row(8 * N - 2) +
                    42.0 * T5(N - 1) * b.row(8 * N - 1));

      // negative jerk
      B2.row(2) = -(6.0 * b.row(8 * N - 5) +
                    24.0 * T1(N - 1) * b.row(8 * N - 4) +
                    60.0 * T2(N - 1) * b.row(8 * N - 3) +
                    120.0 * T3(N - 1) * b.row(8 * N - 2) +
                    210.0 * T4(N - 1) * b.row(8 * N - 1));

      // negative snap
      B2.row(3) = -(24.0 * b.row(8 * N - 4) +
                    120.0 * T1(N - 1) * b.row(8 * N - 3) +
                    360.0 * T2(N - 1) * b.row(8 * N - 2) +
                    840.0 * T3(N - 1) * b.row(8 * N - 1));

      gdT(N - 1) += B2.cwiseProduct(adjGrad.block<4, Dim>(8 * N - 4, 0)).sum();

      return;
    }
  };

  template <int Dim>  class MinJerkOpt
  {
    public:
        int N;
        BandedSystem A;
        Eigen::MatrixXd c;

        Eigen::VectorXd T1;
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;
        Eigen::VectorXd T4;
        Eigen::VectorXd T5;

        Eigen::DiagonalMatrix<double, Dim> energy_weights;

    public:
        MinJerkOpt() = default;
        ~MinJerkOpt() { A.destroy(); }

        inline void reset(const int &pieceNum)
        {
            N = pieceNum;
            A.create(6 * N, 6, 6);
            c.resize(6 * N, Dim);
            T1.resize(N);
            T2.resize(N);
            T3.resize(N);
            T4.resize(N);
            T5.resize(N);
            Eigen::VectorXd ew;
            ew.resize(Dim);
            ew.setConstant(1.0);
            energy_weights = ew.asDiagonal();
            return;
        }

        inline void reset(const int &pieceNum, const Eigen::VectorXd &ew)
        {
            assert(ew.size() == Dim);
            N = pieceNum;
            A.create(6 * N, 6, 6);
            c.resize(6 * N, Dim);
            T1.resize(N);
            T2.resize(N);
            T3.resize(N);
            T4.resize(N);
            T5.resize(N);
            energy_weights = ew.asDiagonal();
            return;
        }

        // from q,T to c,T
        inline void generate(const Eigen::MatrixXd &headPVA,
                            const Eigen::MatrixXd &tailPVA,
                            const Eigen::MatrixXd &inPs,
                            const Eigen::VectorXd &ts)
        {
            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);
            T4 = T2.cwiseProduct(T2);
            T5 = T4.cwiseProduct(T1);

            A.reset();
            c.setZero();

            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            A(2, 2) = 2.0;
            c.row(0) = headPVA.col(0).transpose();
            c.row(1) = headPVA.col(1).transpose();
            c.row(2) = headPVA.col(2).transpose();

            for (int i = 0; i < N - 1; i++)
            {
                A(6 * i + 3, 6 * i + 3) = 6.0;
                A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
                A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
                A(6 * i + 3, 6 * i + 9) = -6.0;
                A(6 * i + 4, 6 * i + 4) = 24.0;
                A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
                A(6 * i + 4, 6 * i + 10) = -24.0;
                A(6 * i + 5, 6 * i) = 1.0;
                A(6 * i + 5, 6 * i + 1) = T1(i);
                A(6 * i + 5, 6 * i + 2) = T2(i);
                A(6 * i + 5, 6 * i + 3) = T3(i);
                A(6 * i + 5, 6 * i + 4) = T4(i);
                A(6 * i + 5, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i) = 1.0;
                A(6 * i + 6, 6 * i + 1) = T1(i);
                A(6 * i + 6, 6 * i + 2) = T2(i);
                A(6 * i + 6, 6 * i + 3) = T3(i);
                A(6 * i + 6, 6 * i + 4) = T4(i);
                A(6 * i + 6, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i + 6) = -1.0;
                A(6 * i + 7, 6 * i + 1) = 1.0;
                A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
                A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
                A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
                A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
                A(6 * i + 7, 6 * i + 7) = -1.0;
                A(6 * i + 8, 6 * i + 2) = 2.0;
                A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
                A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
                A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
                A(6 * i + 8, 6 * i + 8) = -2.0;

                c.row(6 * i + 5) = inPs.col(i).transpose();
            }

            A(6 * N - 3, 6 * N - 6) = 1.0;
            A(6 * N - 3, 6 * N - 5) = T1(N - 1);
            A(6 * N - 3, 6 * N - 4) = T2(N - 1);
            A(6 * N - 3, 6 * N - 3) = T3(N - 1);
            A(6 * N - 3, 6 * N - 2) = T4(N - 1);
            A(6 * N - 3, 6 * N - 1) = T5(N - 1);
            A(6 * N - 2, 6 * N - 5) = 1.0;
            A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
            A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
            A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
            A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
            A(6 * N - 1, 6 * N - 4) = 2;
            A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
            A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
            A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

            c.row(6 * N - 3) = tailPVA.col(0).transpose();
            c.row(6 * N - 2) = tailPVA.col(1).transpose();
            c.row(6 * N - 1) = tailPVA.col(2).transpose();

            A.factorizeLU();
            A.solve(c);

            return;
        }

        inline PolyTrajectory<Dim, 5> getTraj() const
        {
            PolyTrajectory<Dim, 5> polytraj;
            polytraj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                polytraj.emplace_back(T1(i),
                                    c.block<6, Dim>(6 * i, 0)
                                        .transpose()
                                        .rowwise()
                                        .reverse());
            }
            return polytraj;
        }

        inline double getTrajJerkCost() const
        {
            double energy = 0.0;
            for (int i = 0; i < N; i++)
            {
                // energy += 36.0 * c.row(6 * i + 3).squaredNorm() * T1(i) +
                //             144.0 * c.row(6 * i + 4).dot(c.row(6 * i + 3)) * T2(i) +
                //             192.0 * c.row(6 * i + 4).squaredNorm() * T3(i) +
                //             240.0 * c.row(6 * i + 5).dot(c.row(6 * i + 3)) * T3(i) +
                //             720.0 * c.row(6 * i + 5).dot(c.row(6 * i + 4)) * T4(i) +
                //             720.0 * c.row(6 * i + 5).squaredNorm() * T5(i);
                energy += 36.0  * (c.row(6 * i + 3) * energy_weights).dot(c.row(6 * i + 3)) * T1(i) +
                          144.0 * (c.row(6 * i + 4) * energy_weights).dot(c.row(6 * i + 3)) * T2(i) +
                          192.0 * (c.row(6 * i + 4) * energy_weights).dot(c.row(6 * i + 4)) * T3(i) +
                          240.0 * (c.row(6 * i + 5) * energy_weights).dot(c.row(6 * i + 3)) * T3(i) +
                          720.0 * (c.row(6 * i + 5) * energy_weights).dot(c.row(6 * i + 4)) * T4(i) +
                          720.0 * (c.row(6 * i + 5) * energy_weights).dot(c.row(6 * i + 5)) * T5(i);

            }
            return energy;
        }

        inline const Eigen::MatrixXd &getCoeffs(void) const
        {
            return c;
        }

        // know J=∫j²dt
        // then get ∂J/∂c, ∂J/∂T 
        inline void calJerkGradCT(Eigen::MatrixXd& gdC, Eigen::VectorXd &gdT) 
        {
            gdC.resize(6 * N, Dim); 
            for (int i = 0; i < N; i++)
            {
                // gdC.row(6 * i + 5) = 240.0 * c.row(6 * i + 3) * T3(i) +
                //                         720.0 * c.row(6 * i + 4) * T4(i) +
                //                         1440.0 * c.row(6 * i + 5) * T5(i);
                // gdC.row(6 * i + 4) = 144.0 * c.row(6 * i + 3) * T2(i) +
                //                         384.0 * c.row(6 * i + 4) * T3(i) +
                //                         720.0 * c.row(6 * i + 5) * T4(i);
                // gdC.row(6 * i + 3) = 72.0 * c.row(6 * i + 3) * T1(i) +
                //                         144.0 * c.row(6 * i + 4) * T2(i) +
                //                         240.0 * c.row(6 * i + 5) * T3(i);
                // gdC.block<3, Dim>(6 * i, 0).setZero();
                gdC.row(6 * i + 5) = 240.0  * c.row(6 * i + 3) * energy_weights * T3(i) +
                                     720.0  * c.row(6 * i + 4) * energy_weights * T4(i) +
                                     1440.0 * c.row(6 * i + 5) * energy_weights * T5(i);
                gdC.row(6 * i + 4) = 144.0 * c.row(6 * i + 3) * energy_weights * T2(i) +
                                     384.0 * c.row(6 * i + 4) * energy_weights * T3(i) +
                                     720.0 * c.row(6 * i + 5) * energy_weights * T4(i);
                gdC.row(6 * i + 3) = 72.0  * c.row(6 * i + 3) * energy_weights * T1(i) +
                                     144.0 * c.row(6 * i + 4) * energy_weights * T2(i) +
                                     240.0 * c.row(6 * i + 5) * energy_weights * T3(i);
                gdC.block<3, Dim>(6 * i, 0).setZero();
            }

            gdT.resize(N);
            for (int i = 0; i < N; i++)
            {
                // gdT(i) = 36.0 * c.row(6 * i + 3).squaredNorm() +
                //             288.0 * c.row(6 * i + 4).dot(c.row(6 * i + 3)) * T1(i) +
                //             576.0 * c.row(6 * i + 4).squaredNorm() * T2(i) +
                //             720.0 * c.row(6 * i + 5).dot(c.row(6 * i + 3)) * T2(i) +
                //             2880.0 * c.row(6 * i + 5).dot(c.row(6 * i + 4)) * T3(i) +
                //             3600.0 * c.row(6 * i + 5).squaredNorm() * T4(i);
                gdT(i) = 36.0   * (c.row(6 * i + 3) * energy_weights).dot(c.row(6 * i + 3)) +
                         288.0  * (c.row(6 * i + 4) * energy_weights).dot(c.row(6 * i + 3)) * T1(i) +
                         576.0  * (c.row(6 * i + 4) * energy_weights).dot(c.row(6 * i + 4)) * T2(i) +
                         720.0  * (c.row(6 * i + 5) * energy_weights).dot(c.row(6 * i + 3)) * T2(i) +
                         2880.0 * (c.row(6 * i + 5) * energy_weights).dot(c.row(6 * i + 4)) * T3(i) +
                         3600.0 * (c.row(6 * i + 5) * energy_weights).dot(c.row(6 * i + 5)) * T4(i);
            }
            return;
        }

        // know ∂K/∂C, ∂K/∂T, K(C(q,T),T) = W(q,T)
        // then get ∂W/∂q, ∂W/∂T
        inline void calGradCTtoQT(const Eigen::MatrixXd& gdC, Eigen::VectorXd& gdT, Eigen::MatrixXd& gdP, \
                              Eigen::MatrixXd& gdTail)

        {
            gdP.resize(Dim, N - 1);
            Eigen::MatrixXd adjGrad = gdC;
            A.solveAdj(adjGrad);

            for (int i = 0; i < N - 1; i++)
            {
                gdP.col(i) = adjGrad.row(6 * i + 5).transpose();
            }

            gdTail.resize(Dim, 3);
            gdTail = adjGrad.bottomRows(3).transpose();

            Eigen::Matrix<double, 6, Dim> B1;
            Eigen::Matrix<double, 3, Dim> B2;
            for (int i = 0; i < N - 1; i++)
            {
                // negative velocity
                B1.row(2) = -(c.row(i * 6 + 1) +
                                2.0 * T1(i) * c.row(i * 6 + 2) +
                                3.0 * T2(i) * c.row(i * 6 + 3) +
                                4.0 * T3(i) * c.row(i * 6 + 4) +
                                5.0 * T4(i) * c.row(i * 6 + 5));
                B1.row(3) = B1.row(2);

                // negative acceleration
                B1.row(4) = -(2.0 * c.row(i * 6 + 2) +
                                6.0 * T1(i) * c.row(i * 6 + 3) +
                                12.0 * T2(i) * c.row(i * 6 + 4) +
                                20.0 * T3(i) * c.row(i * 6 + 5));

                // negative jerk
                B1.row(5) = -(6.0 * c.row(i * 6 + 3) +
                                24.0 * T1(i) * c.row(i * 6 + 4) +
                                60.0 * T2(i) * c.row(i * 6 + 5));

                // negative snap
                B1.row(0) = -(24.0 * c.row(i * 6 + 4) +
                                120.0 * T1(i) * c.row(i * 6 + 5));

                // negative crackle
                B1.row(1) = -120.0 * c.row(i * 6 + 5);

                gdT(i) += B1.cwiseProduct(adjGrad.block<6, Dim>(6 * i + 3, 0)).sum();
            }

            // negative velocity
            B2.row(0) = -(c.row(6 * N - 5) +
                            2.0 * T1(N - 1) * c.row(6 * N - 4) +
                            3.0 * T2(N - 1) * c.row(6 * N - 3) +
                            4.0 * T3(N - 1) * c.row(6 * N - 2) +
                            5.0 * T4(N - 1) * c.row(6 * N - 1));

            // negative acceleration
            B2.row(1) = -(2.0 * c.row(6 * N - 4) +
                            6.0 * T1(N - 1) * c.row(6 * N - 3) +
                            12.0 * T2(N - 1) * c.row(6 * N - 2) +
                            20.0 * T3(N - 1) * c.row(6 * N - 1));

            // negative jerk
            B2.row(2) = -(6.0 * c.row(6 * N - 3) +
                            24.0 * T1(N - 1) * c.row(6 * N - 2) +
                            60.0 * T2(N - 1) * c.row(6 * N - 1));

            gdT(N - 1) += B2.cwiseProduct(adjGrad.block<3, Dim>(6 * N - 3, 0)).sum();
            return;
        }
  };

  template <int Dim>  class MinAccOpt
  {
    public:
        int N;
        BandedSystem A;
        Eigen::MatrixXd c;

        Eigen::VectorXd T1;
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;

    public:
        MinAccOpt() = default;
        ~MinAccOpt() { A.destroy(); }

        inline void reset(const int &pieceNum)
        {
            N = pieceNum;
            A.create(4 * N, 4, 4);
            c.resize(4 * N, Dim);
            T1.resize(N);
            T2.resize(N);
            T3.resize(N);
            return;
        }

        // from q,T to c,T
        inline void generate(const Eigen::MatrixXd &headPV,
                            const Eigen::MatrixXd &tailPV,
                            const Eigen::MatrixXd &inPs,
                            const Eigen::VectorXd &ts)
        {
            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);

            A.reset();
            c.setZero();

            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            c.row(0) = headPV.col(0).transpose();
            c.row(1) = headPV.col(1).transpose();

            for (int i = 0; i < N - 1; i++)
            {
                A(4 * i + 2, 4 * i + 2) = 2.0;
                A(4 * i + 2, 4 * i + 3) = 6.0 * T1(i);
                A(4 * i + 2, 4 * i + 6) = -2.0;
                A(4 * i + 3, 4 * i) = 1.0;
                A(4 * i + 3, 4 * i + 1) = T1(i);
                A(4 * i + 3, 4 * i + 2) = T2(i);
                A(4 * i + 3, 4 * i + 3) = T3(i);
                A(4 * i + 4, 4 * i) = 1.0;
                A(4 * i + 4, 4 * i + 1) = T1(i);
                A(4 * i + 4, 4 * i + 2) = T2(i);
                A(4 * i + 4, 4 * i + 3) = T3(i);
                A(4 * i + 4, 4 * i + 4) = -1.0;
                A(4 * i + 5, 4 * i + 1) = 1.0;
                A(4 * i + 5, 4 * i + 2) = 2.0 * T1(i);
                A(4 * i + 5, 4 * i + 3) = 3.0 * T2(i);
                A(4 * i + 5, 4 * i + 5) = -1.0;

                c.row(4 * i + 3) = inPs.col(i).transpose();
            }

            A(4 * N - 2, 4 * N - 4) = 1.0;
            A(4 * N - 2, 4 * N - 3) = T1(N - 1);
            A(4 * N - 2, 4 * N - 2) = T2(N - 1);
            A(4 * N - 2, 4 * N - 1) = T3(N - 1);
            A(4 * N - 1, 4 * N - 3) = 1.0;
            A(4 * N - 1, 4 * N - 2) = 2 * T1(N - 1);
            A(4 * N - 1, 4 * N - 1) = 3 * T2(N - 1);

            c.row(4 * N - 2) = tailPV.col(0).transpose();
            c.row(4 * N - 1) = tailPV.col(1).transpose();

            A.factorizeLU();
            A.solve(c);

            return;
        }

        inline PolyTrajectory<Dim, 3> getTraj() const
        {
            PolyTrajectory<Dim, 3> polytraj;
            polytraj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                polytraj.emplace_back(T1(i),
                                    c.block<4, Dim>(4 * i, 0)
                                        .transpose()
                                        .rowwise()
                                        .reverse());
            }
            return polytraj;
        }

        inline double getTrajAccCost() const
        {
            double energy = 0.0;
            for (int i = 0; i < N; i++)
            {
                energy += 4.0 * c.row(4 * i + 2).squaredNorm() * T1(i) +
                          12.0 * c.row(4 * i + 2).dot(c.row(4 * i + 3)) * T2(i) +
                          12.0 * c.row(4 * i + 3).squaredNorm() * T3(i);
            }
            return energy;
        }

        inline const Eigen::MatrixXd &getCoeffs(void) const
        {
            return c;
        }

        // know J=∫j²dt
        // then get ∂J/∂c, ∂J/∂T 
        inline void calAccGradCT(Eigen::MatrixXd& gdC, Eigen::VectorXd &gdT) 
        {
            gdC.resize(4 * N, Dim); 
            for (int i = 0; i < N; i++)
            {
                gdC.row(4 * i + 3) = 12.0 * c.row(4 * i + 2) * T2(i) +
                                     24.0 * c.row(4 * i + 3) * T3(i);
                gdC.row(4 * i + 2) = 8.0 * c.row(4 * i + 2) * T1(i) +
                                     12.0 * c.row(4 * i + 3) * T2(i);
                gdC.block<2, Dim>(4 * i, 0).setZero();
            }

            gdT.resize(N);
            for (int i = 0; i < N; i++)
            {
                gdT(i) = 4.0 * c.row(4 * i + 2).squaredNorm() +
                         24.0 * c.row(4 * i + 2).dot(c.row(4 * i + 3)) * T1(i) +
                         36.0 * c.row(4 * i + 3).squaredNorm() * T2(i);
            }
            return;
        }

        // know ∂K/∂C, ∂K/∂T, K(C(q,T),T) = W(q,T)
        // then get ∂W/∂q, ∂W/∂T
        inline void calGradCTtoQT(const Eigen::MatrixXd& gdC, Eigen::VectorXd& gdT, Eigen::MatrixXd& gdP, \
                              Eigen::MatrixXd& gdTail)

        {
            gdP.resize(Dim, N - 1);
            Eigen::MatrixXd adjGrad = gdC;
            A.solveAdj(adjGrad);

            for (int i = 0; i < N - 1; i++)
            {
                gdP.col(i) = adjGrad.row(4 * i + 3).transpose();
            }

            gdTail.resize(Dim, 2);
            gdTail = adjGrad.bottomRows(2).transpose();

            Eigen::Matrix<double, 4, Dim> B1;
            Eigen::Matrix<double, 2, Dim> B2;
            for (int i = 0; i < N - 1; i++)
            {
                // negative jerk
                B1.row(0) = -6.0 * c.row(i * 4 + 3);

                // negative velocity
                B1.row(1) = -(c.row(i * 4 + 1) +
                              2.0 * T1(i) * c.row(i * 4 + 2) +
                              3.0 * T2(i) * c.row(i * 4 + 3));
                B1.row(2) = B1.row(1);

                // negative acceleration
                B1.row(3) = -(2.0 * c.row(i * 4 + 2) +
                              6.0 * T1(i) * c.row(i * 4 + 3));

                gdT(i) += B1.cwiseProduct(adjGrad.block<4, Dim>(4 * i + 2, 0)).sum();
            }

            // negative velocity
            B2.row(0) = -(c.row(4 * N - 3) +
                          2.0 * T1(N - 1) * c.row(4 * N - 2) +
                          3.0 * T2(N - 1) * c.row(4 * N - 1));

            // negative acceleration
            B2.row(1) = -(2.0 * c.row(4 * N - 2) +
                          6.0 * T1(N - 1) * c.row(4 * N - 1));

            gdT(N - 1) += B2.cwiseProduct(adjGrad.block<2, Dim>(4 * N - 2, 0)).sum();
            return;
        }
  };

    template <int Dim>  class MinJerkOptUni
    {
        public:
            int N;
            BandedSystem A;
            Eigen::MatrixXd headPVA, tailPVA;
            Eigen::MatrixXd b, c, adjScaledGrad;
            Eigen::VectorXd t, tInv;

        public:
            MinJerkOptUni() = default;
            ~MinJerkOptUni() { A.destroy(); }

            inline void reset(const int &pieceNum)
            {
                N = pieceNum;
                A.create(6 * N, 6, 6);
                b.resize(6 * N, Dim);
                c.resize(6 * N, Dim);
                adjScaledGrad.resize(6 * N, Dim);

                t(0) = 1.0;

                A(0, 0) = 1.0;
                A(1, 1) = 1.0;
                A(2, 2) = 2.0;
                for (int i = 0; i < N - 1; i++) 
                {
                    A(6 * i + 3, 6 * i + 3) = 6.0;
                    A(6 * i + 3, 6 * i + 4) = 24.0;
                    A(6 * i + 3, 6 * i + 5) = 60.0;
                    A(6 * i + 3, 6 * i + 9) = -6.0;
                    A(6 * i + 4, 6 * i + 4) = 24.0;
                    A(6 * i + 4, 6 * i + 5) = 120.0;
                    A(6 * i + 4, 6 * i + 10) = -24.0;
                    A(6 * i + 5, 6 * i) = 1.0;
                    A(6 * i + 5, 6 * i + 1) = 1.0;
                    A(6 * i + 5, 6 * i + 2) = 1.0;
                    A(6 * i + 5, 6 * i + 3) = 1.0;
                    A(6 * i + 5, 6 * i + 4) = 1.0;
                    A(6 * i + 5, 6 * i + 5) = 1.0;
                    A(6 * i + 6, 6 * i) = 1.0;
                    A(6 * i + 6, 6 * i + 1) = 1.0;
                    A(6 * i + 6, 6 * i + 2) = 1.0;
                    A(6 * i + 6, 6 * i + 3) = 1.0;
                    A(6 * i + 6, 6 * i + 4) = 1.0;
                    A(6 * i + 6, 6 * i + 5) = 1.0;
                    A(6 * i + 6, 6 * i + 6) = -1.0;
                    A(6 * i + 7, 6 * i + 1) = 1.0;
                    A(6 * i + 7, 6 * i + 2) = 2.0;
                    A(6 * i + 7, 6 * i + 3) = 3.0;
                    A(6 * i + 7, 6 * i + 4) = 4.0;
                    A(6 * i + 7, 6 * i + 5) = 5.0;
                    A(6 * i + 7, 6 * i + 7) = -1.0;
                    A(6 * i + 8, 6 * i + 2) = 2.0;
                    A(6 * i + 8, 6 * i + 3) = 6.0;
                    A(6 * i + 8, 6 * i + 4) = 12.0;
                    A(6 * i + 8, 6 * i + 5) = 20.0;
                    A(6 * i + 8, 6 * i + 8) = -2.0;
                }
                A(6 * N - 3, 6 * N - 6) = 1.0;
                A(6 * N - 3, 6 * N - 5) = 1.0;
                A(6 * N - 3, 6 * N - 4) = 1.0;
                A(6 * N - 3, 6 * N - 3) = 1.0;
                A(6 * N - 3, 6 * N - 2) = 1.0;
                A(6 * N - 3, 6 * N - 1) = 1.0;
                A(6 * N - 2, 6 * N - 5) = 1.0;
                A(6 * N - 2, 6 * N - 4) = 2.0;
                A(6 * N - 2, 6 * N - 3) = 3.0;
                A(6 * N - 2, 6 * N - 2) = 4.0;
                A(6 * N - 2, 6 * N - 1) = 5.0;
                A(6 * N - 1, 6 * N - 4) = 2.0;
                A(6 * N - 1, 6 * N - 3) = 6.0;
                A(6 * N - 1, 6 * N - 2) = 12.0;
                A(6 * N - 1, 6 * N - 1) = 20.0;
                A.factorizeLU();

                return;
            }

            // from q,T to c,T
            inline void generate(const Eigen::MatrixXd &headPVA,
                                const Eigen::MatrixXd &tailPVA,
                                const Eigen::MatrixXd &inPs,
                                const double &dT)
            {
                headPVA = headPVA;
                tailPVA = tailPVA;

                t(1) = dT;
                t(2) = t(1) * t(1);
                t(3) = t(2) * t(1);
                t(4) = t(2) * t(2);
                t(5) = t(4) * t(1);
                tInv = t.cwiseInverse();

                b.setZero();
                b.row(0) = headPVA.col(0).transpose();
                b.row(1) = headPVA.col(1).transpose() * t(1);
                b.row(2) = headPVA.col(2).transpose() * t(2);
                for (int i = 0; i < N - 1; i++) 
                {
                    b.row(6 * i + 5) = inPs.col(i).transpose();
                }
                b.row(6 * N - 3) = tailPVA.col(0).transpose();
                b.row(6 * N - 2) = tailPVA.col(1).transpose() * t(1);
                b.row(6 * N - 1) = tailPVA.col(2).transpose() * t(2);

                A.solve(b);
                for (int i = 0; i < N; i++) 
                {
                    c.block<6, Dim>(6 * i, 0) =
                        b.block<6, Dim>(6 * i, 0).array().colwise() * tInv.array();
                }
                return;
            }

            inline PolyTrajectory<Dim, 5> getTraj() const
            {
                PolyTrajectory<Dim, 5> polytraj;
                polytraj.reserve(N);
                for (int i = 0; i < N; i++)
                {
                    polytraj.emplace_back(t(1), c.block<6, Dim>(6 * i, 0).transpose().rowwise().reverse());
                }

                return polytraj;
            }

            inline double getTrajJerkCost() const
            {
                double energy = 0.0;
                for (int i = 0; i < N; i++) 
                {
                    energy += 36.0 * c.row(6 * i + 3).squaredNorm() * t(1) +
                            144.0 * c.row(6 * i + 4).dot(c.row(6 * i + 3)) * t(2) +
                            192.0 * c.row(6 * i + 4).squaredNorm() * t(3) +
                            240.0 * c.row(6 * i + 5).dot(c.row(6 * i + 3)) * t(3) +
                            720.0 * c.row(6 * i + 5).dot(c.row(6 * i + 4)) * t(4) +
                            720.0 * c.row(6 * i + 5).squaredNorm() * t(5);
                }
                return energy;
            }

            inline const Eigen::MatrixXd &getCoeffs(void) const
            {
                return c;
            }

            // know J=∫j²dt
            // then get ∂J/∂c, ∂J/∂T 
            inline void calJerkGradCT(Eigen::MatrixXd& gdC, double &gdT) 
            {
                gdC.resize(6 * N, Dim); 
                for (int i = 0; i < N; i++) 
                {
                    gdC.row(6 * i + 5) = 240.0 * c.row(6 * i + 3) * t(3) +
                                        720.0 * c.row(6 * i + 4) * t(4) +
                                        1440.0 * c.row(6 * i + 5) * t(5);
                    gdC.row(6 * i + 4) = 144.0 * c.row(6 * i + 3) * t(2) +
                                        384.0 * c.row(6 * i + 4) * t(3) +
                                        720.0 * c.row(6 * i + 5) * t(4);
                    gdC.row(6 * i + 3) = 72.0 * c.row(6 * i + 3) * t(1) +
                                        144.0 * c.row(6 * i + 4) * t(2) +
                                        240.0 * c.row(6 * i + 5) * t(3);
                    gdC.block<3, Dim>(6 * i, 0).setZero();
                }
                gdT = 0.0;
                for (int i = 0; i < N; i++) 
                {
                    gdT += 36.0 * c.row(6 * i + 3).squaredNorm() +
                            288.0 * c.row(6 * i + 4).dot(c.row(6 * i + 3)) * t(1) +
                            576.0 * c.row(6 * i + 4).squaredNorm() * t(2) +
                            720.0 * c.row(6 * i + 5).dot(c.row(6 * i + 3)) * t(2) +
                            2880.0 * c.row(6 * i + 5).dot(c.row(6 * i + 4)) * t(3) +
                            3600.0 * c.row(6 * i + 5).squaredNorm() * t(4);
                }
                return;
            }

            // know ∂K/∂C, ∂K/∂T, K(C(q,T),T) = W(q,T)
            // then get ∂W/∂q, ∂W/∂T
            inline void calGradCTtoQT(const Eigen::MatrixXd& gdC, double& gdT, Eigen::MatrixXd& gdP, \
                                Eigen::MatrixXd& gdTail)

            {
                gdP.resize(Dim, N - 1);
                gdTail.resize(Dim, 3);
                for (int i = 0; i < N; i++) 
                {
                    adjScaledGrad.block<6, Dim>(6 * i, 0) =
                        gdC.block<6, Dim>(6 * i, 0).array().colwise() * tInv.array();
                }
                A.solveAdj(adjScaledGrad);

                for (int i = 0; i < N - 1; i++) 
                {
                    gdP.col(i) = adjScaledGrad.row(6 * i + 5).transpose();
                }
                gdTail = adjScaledGrad.bottomRows(3).transpose() * t.head<3>().asDiagonal();

                gdT += headPVA.col(1).dot(adjScaledGrad.row(1));
                gdT += headPVA.col(2).dot(adjScaledGrad.row(2)) * 2.0 * t(1);
                gdT += tailPVA.col(1).dot(adjScaledGrad.row(6 * N - 2));
                gdT += tailPVA.col(2).dot(adjScaledGrad.row(6 * N - 1)) * 2.0 * t(1);
                Eigen::Matrix<double, 6, 1> gdtInv;
                gdtInv(0) = 0.0;
                gdtInv(1) = -1.0 * tInv(2);
                gdtInv(2) = -2.0 * tInv(3);
                gdtInv(3) = -3.0 * tInv(4);
                gdtInv(4) = -4.0 * tInv(5);
                gdtInv(5) = -5.0 * tInv(5) * tInv(1);
                const Eigen::VectorXd gdcol = gdC.cwiseProduct(b).rowwise().sum();
                for (int i = 0; i < N; i++) 
                {
                    gdT += gdtInv.dot(gdcol.segment<6>(6 * i));
                }
                return;
            }
    };

}