# PCL 实现 icp 算法原理源码解析

## icp 算法流程

icp 算法可以使用 SVD 分解进行求解，在 PCL 中也是这么实现的，首先看一下 icp 算法使用 SVD 分解的流程：

1. 给定两幅点云：
    - $P$（source）
    - $Q$（target）
2. 获取两幅点云之间的匹配关系
3. 计算旋转矩阵 $R$，平移向量 $t$
    1. 首先计算两幅点云的质心：$\hat{p}$，$\hat{q}$
    2. 计算两幅点云在减去质心之后对应新的点云 $P'$ 和 $Q$’
    3. 进行 SVD 分解：$P' Q'^T = U\sum V^T$
    4. $R = VU^T$, $t=\hat{q}-R\hat{p}$
4. 重复 2、3 步骤，若迭代次数达到设定值或 $R$、$t$ 变化小于阈值，则停止迭代

## PCL 实现 icp 算法

在 PCL 中，icp 算法也是通过 SVD 实现的，下面的函数是从源码中摘取的部分函数，函数内部的一些代码大多数被删掉了，只保留了一些核心的内容，当然看起来有一些变量存在未定义行为，但不妨碍理解整个实现流程，如果觉得不适应，请移步源码。

### align

align 用于点云配准的计算函数，是点云配准的入口，里面包含核心函数 computeTransformation。

```cpp
void pcl::Registration<PointSource, PointTarget, Scalar>::align(PointCloudSource &output, const Matrix4& guess)
{
    computeTransformation (output, guess);
}
```

### computeTransformation

这是计算旋转矩阵的核心函数，函数体内有一个循环，就是通过该循环进行不断的迭代，求取最终的变换矩阵。

```cpp
void pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::computeTransformation(PointCloudSource &output, const Matrix4 &guess)
{
  PointCloudSourcePtr input_transformed (new PointCloudSource);

  nr_iterations_ = 0;
  converged_ = false;

  // Initialise final transformation to the guessed one
  final_transformation_ = guess;

  // 循环开始
  do
  {
    // 计算两幅点云的对应关系		关键函数 1
    correspondence_estimation_->determineCorrespondences (*correspondences_, corr_dist_threshold_);

    // 根据对应关系求取变换矩阵		关键函数 2
    transformation_estimation_->estimateRigidTransformation (*input_transformed, *target_, *correspondences_, transformation_);

    // 将变换矩阵作用于source点云
    transformCloud (*input_transformed, *input_transformed, transformation_);

    // 更新最终的变换矩阵    
    final_transformation_ = transformation_ * final_transformation_;

    ++nr_iterations_;

    converged_ = static_cast<bool> ((*convergence_criteria_));
  }
  while (!converged_);
    
  // 最终得到配准后的点云output
  transformCloud (*input_, output, final_transformation_);
}
```

在该函数内部，有几个重要的函数，它们实现了必要的功能，下面展开说一下：

#### 1.determineCorrespondences

该函数用于求取两幅点云的对应关系，主要流程如下：

1. 将 target 点云加入 kdtree
2. 遍历 source 点云中的每一个点，寻找对应 target 点云中的最近点
3. 将以上一组点云作为一对匹配关系保存下来

```cpp
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::determineCorrespondences (pcl::Correspondences &correspondences, double max_distance)
{
    double max_dist_sqr = max_distance * max_distance;

    correspondences.resize (indices_->size ());

    std::vector<int> index (1);
    std::vector<float> distance (1);
    pcl::Correspondence corr;
    unsigned int nr_valid_correspondences = 0;

    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
        tree_->nearestKSearch (input_->points[*idx], 1, index, distance);
        if (distance[0] > max_dist_sqr)
            continue;

        corr.index_query = *idx;
        corr.index_match = index[0];
        corr.distance = distance[0];
        correspondences[nr_valid_correspondences++] = corr;
    }

    correspondences.resize (nr_valid_correspondences);
}
```

#### 2.estimateRigidTransformation

该函数用于进行刚体变换，内部有两个核心函数。

```cpp
void pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const pcl::Correspondences &correspondences,
    Matrix4 &transformation_matrix) const
{
  ConstCloudIterator<PointSource> source_it (cloud_src, correspondences, true);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt, correspondences, false);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}
```

**ConstCloudIterator**

该函数被调用了两次，目的是将 source 与 target 内的点根据对应关系对应起来，即通过索引直接对应，source[i] 对应 target[i]。

```cpp
template <class PointT>
pcl::ConstCloudIterator<PointT>::ConstCloudIterator (
    const PointCloud<PointT>& cloud, const Correspondences& corrs, bool source)
{
    std::vector<int> indices;
    indices.reserve (corrs.size ());
    if (source)
    {
        for (typename Correspondences::const_iterator indexIt = corrs.begin (); indexIt != corrs.end (); ++indexIt)
            indices.push_back (indexIt->index_query);
    }
    else
    {
        for (typename Correspondences::const_iterator indexIt = corrs.begin (); indexIt != corrs.end (); ++indexIt)
            indices.push_back (indexIt->index_match);
    }
    iterator_ = new typename pcl::ConstCloudIterator<PointT>::ConstIteratorIdx (cloud, indices);
}
```

**estimateRigidTransformation**

该函数是为最终的 SVD 分解做准备，首先要求两幅点云各自的质心，然后计算两幅点云去质心之后的新点云，然后传入最后的函数。

```cpp
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    ConstCloudIterator<PointSource>& source_it,
    ConstCloudIterator<PointTarget>& target_it,
    Matrix4 &transformation_matrix) const
{
    transformation_matrix.setIdentity ();

    Eigen::Matrix<Scalar, 4, 1> centroid_src, centroid_tgt;
    // 计算质心
    compute3DCentroid (source_it, centroid_src);
    compute3DCentroid (target_it, centroid_tgt);
    source_it.reset (); target_it.reset ();

    // 减去质心
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> cloud_src_demean, cloud_tgt_demean;
    demeanPointCloud (source_it, centroid_src, cloud_src_demean);
    demeanPointCloud (target_it, centroid_tgt, cloud_tgt_demean);

    // SVD 分解
    getTransformationFromCorrelation (cloud_src_demean, centroid_src, cloud_tgt_demean, centroid_tgt, transformation_matrix);
}
```

**getTransformationFromCorrelation**

该函数就是最终的函数了，通过 SVD 分解求出了 R 和 t，SVD 分解则是调用了 Eigen 内的函数。

```cpp
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>::getTransformationFromCorrelation (
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_src_demean,
    const Eigen::Matrix<Scalar, 4, 1> &centroid_src,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &cloud_tgt_demean,
    const Eigen::Matrix<Scalar, 4, 1> &centroid_tgt,
    Matrix4 &transformation_matrix) const
{
  transformation_matrix.setIdentity ();

  // Assemble the correlation matrix H = source * target'
  Eigen::Matrix<Scalar, 3, 3> H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner (3, 3);

  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3> > svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<Scalar, 3, 3> u = svd.matrixU ();
  Eigen::Matrix<Scalar, 3, 3> v = svd.matrixV ();

  // Compute R = V * U'
  if (u.determinant () * v.determinant () < 0)
  {
    for (int x = 0; x < 3; ++x)
      v (x, 2) *= -1;
  }

  Eigen::Matrix<Scalar, 3, 3> R = v * u.transpose ();

  // Return the correct transformation
  transformation_matrix.topLeftCorner (3, 3) = R;
  const Eigen::Matrix<Scalar, 3, 1> Rc (R * centroid_src.head (3));
  transformation_matrix.block (0, 3, 3, 1) = centroid_tgt.head (3) - Rc;
}
```

## 总结

icp 算法的流程如上所述，在 PCL 中也是这样实现的，不过源码中具体的函数分工很细，不断的跳着查看挺麻烦，但是如果对流程理解了，具体看源码也没那么难了。