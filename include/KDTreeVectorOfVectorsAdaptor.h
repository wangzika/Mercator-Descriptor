/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-16 Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

#pragma once

#include <nanoflann.hpp>

#include <vector>

// ===== This example shows how to use nanoflann with these types of containers: =======
//typedef std::vector<std::vector<double> > my_vector_of_vectors_t;
//typedef std::vector<Eigen::VectorXd> my_vector_of_vectors_t;   // This requires #include <Eigen/Dense>
// =====================================================================================


/** A simple vector-of-vectors adaptor for nanoflann, without duplicating the storage.
  *  The i'th vector represents a point in the state space.
  *
  *  \tparam DIM If set to >0, it specifies a compile-time fixed dimensionality for the points in the data set, allowing more compiler optimizations.
  *  \tparam num_t The type of the point coordinates (typically, double or float).
  *  \tparam Distance The distance metric to use: nanoflann::metric_L1, nanoflann::metric_L2, nanoflann::metric_L2_Simple, etc.
  *  \tparam IndexType The type for indices in the KD-tree index (typically, size_t of int)
  */
template <class VectorOfVectorsType, typename num_t = double, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = size_t>
struct KDTreeVectorOfVectorsAdaptor
{
	typedef KDTreeVectorOfVectorsAdaptor<VectorOfVectorsType,num_t,DIM,Distance> self_t;
	typedef typename Distance::template traits<num_t,self_t>::distance_t metric_t;
	typedef nanoflann::KDTreeSingleIndexAdaptor< metric_t,self_t,DIM,IndexType>  index_t;

	index_t* index; //! The kd-tree index for the user to call its methods as usual with any other FLANN index.

	/// Constructor: takes a const ref to the vector of vectors object with the data points
	KDTreeVectorOfVectorsAdaptor(const size_t /* dimensionality */, const VectorOfVectorsType &mat, const int leaf_max_size = 10) : m_data(mat)
	{
		assert(mat.size() != 0 && mat[0].size() != 0);
		const size_t dims = mat[0].size();
		if (DIM>0 && static_cast<int>(dims) != DIM)
			throw std::runtime_error("Data set dimensionality does not match the 'DIM' template argument");
		index = new index_t( static_cast<int>(dims), *this /* adaptor */, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size ) );
		index->buildIndex();
	}

	~KDTreeVectorOfVectorsAdaptor() {
		delete index;
	}

	const VectorOfVectorsType &m_data;

	/** Query for the \a num_closest closest points to a given point (entered as query_point[0:dim-1]).
	  *  Note that this is a short-cut method for index->findNeighbors().
	  *  The user can also call index->... methods as desired.
	  * \note nChecks_IGNORED is ignored but kept for compatibility with the original FLANN interface.
	  */

	/// @brief KDTreeVectorOfVectorsAdaptor.h - 查询（const num_t *，const size_t，IndexType *，num_t *，const int）const .query 方法用于查找数据集中给定查询点的 k 个最近邻点。
	/// 在该方法内部，使用指定数量的最近邻居创建了一个 KNNResultSet 对象。 使用 out_indices 和 out_distances_sq 数组作为参数调用 resultSet 对象的 init 方法。
	/// 然后使用 resultSet、query_point 和 SearchParams 对象作为参数调用索引对象的 findNeighbors 方法。 SearchParams 对象用于指定在搜索期间执行的检查次数。
	/// findNeighbors 方法返回后，out_indices 和 out_distances_sq 数组将分别填充 k 近邻的索引和距离。 然后调用函数可以使用这些数组对邻居执行进一步的操作，例如计算它们的平均值或找到最近的邻居。
	/// @param query_point 第一个参数是指向查询点的指针，它是一个包含 num_t 个值的数组。
	/// @param num_closest 第二个参数是要查找的最近邻居的数量。
	/// @param out_indices 第三个参数是指向 IndexType 值数组的指针，该数组将填充 k 个最近邻居的索引。 
	/// @param out_distances_sq 第四个参数是指向 num_t 值数组的指针，该数组将填充查询点与 k 最近邻点之间的平方距离。
	/// @param nChecks_IGNORED 第五个参数是一个可选参数，它指定在搜索过程中要执行的检查次数。
	inline void query(const num_t *query_point, const size_t num_closest, IndexType *out_indices, num_t *out_distances_sq, const int nChecks_IGNORED = 10) const
	{
		nanoflann::KNNResultSet<num_t,IndexType> resultSet(num_closest);
		resultSet.init(out_indices, out_distances_sq);
		index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
	}

	/** @name Interface expected by KDTreeSingleIndexAdaptor
	  * @{ */

	const self_t & derived() const {
		return *this;
	}
	self_t & derived()       {
		return *this;
	}

	// Must return the number of data points
	//kdtree_get_point_count() 常量方法是实现 k-d 树数据结构的类的成员函数。 此方法用于检索存储在 k-d 树中的点数。 
	//const 关键字表示此方法不修改对象的状态。当调用 kdtree_get_point_count() const 方法时，它返回 m_data 向量的大小，
	//其中包含存储在 k-d 树中的点。 其他需要知道 k-d 树中点数的函数或算法使用此方法，例如执行最近邻搜索或遍历树时。
	inline size_t kdtree_get_point_count() const {
		return m_data.size();
	}

	// Returns the dim'th component of the idx'th point in the class:
	inline num_t kdtree_get_pt(const size_t idx, const size_t dim) const {
		return m_data[idx][dim];
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	//kdtree_get_bbox(BBOX &) const 方法的目的是检索 k-d 树的边界框。 BBOX 模板参数表示边界框的类型，它可以是任何提供必要方法和数据成员来定义边界框的类。
	//该函数将对 BBOX 对象的引用作为其参数，该对象将被修改为包含 k-d 树的边界框。 const 限定符表示该函数不修改 k-d 树的内部状态。
	//但是，在提供的代码片段中，kdtree_get_bbox(BBOX &) const 方法的两种实现都只返回 false，表示无法检索边界框。 这表明 k-d 树的实现可能不支持边界框或者功能没有完全实现。
	template <class BBOX>
	bool kdtree_get_bbox(BBOX & /*bb*/) const {
		return false;
	}

	/** @} */

}; // end of KDTreeVectorOfVectorsAdaptor
