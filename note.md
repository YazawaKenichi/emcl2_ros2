# emcl2_node.cpp
## 始点
`emcl2_node.cpp`
``` C++
int main(int argc, char **argv)
{
    //! ノードの立ち上げ
    auto node = std::make_shared<emcl2::EMcl2Node>();
    while(rclcpp::ok())
    {
        //! 周期処理の実行
        node->loop();
        sleep();
    }
    return 0;
}
```

### main 中に実行されるもの
``` C++
emcl2::EMcl2Node()
{
}
```

``` C++
void emcl2::EMcl2Node::loop()
{
    //! 初期化
    if(init_request_)
    {
        pf_->initialize(init_x_, init_y_, init_t_);
    }
    //! 単純リセット
    else if(simple_reset_request_)
    {
        pf_->simpleReset();
    }

    //! なにこれ
    if(init_pf_)
    {
        //! /Odom の取得
        emcl2::EMcl2Node::getOdomPose(x, y, t)
        //! 
        pf_->motionUpdate(x, y, t);
    }
    else
    {
    }
}
```

#### loop 中に実行される内容
``` C++
pf_->initialize()
pf_->simpleReset()
pf_->motionUpdate()
pf_->sensorUpdate()
pf_->meanPose()
publish(OdomFrame, Pose, Particles)
publish(pf_->alpha_)
```

## emcl2_node.cpp まとめ
1. パラメータ用意したりパブサブの準備
2. 主要なループ処理
    a. ノードの初期化
    b. 単純リセット
    c. ExpResetMcl2::pf_ をパブリッシュ
          i. motionUpdate
         ii. sensorUpdate
        iii. meanPose
         iv. alpha_

# ExpResetMcl2
``` C++
class ExpResetMcl2 : public Mcl
```
ということで Mcl を継承している

## sensorUpdate
``` C++
void ExpResetMcl2::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv)
{
    //! スキャンの取得
    for(size_t i = 0; i < scan.ranges_.size(); i++)
    {
        scan.directions_16bit.push_back(Pose::get16bitRepresentation(origin + sgn * i * scan.angle_increment_));
    }

    //! よくわからん処理
    for(auto & p : particles_)
    {
        p.w_ *= p.likelihood(map.get(), scan);
    }

    //! alpha の取得
    alpha_ = nonPenetrationRate(static_cast<int>(particles_.size() * extraction_rate_), map.get(), scan);
    //! いつも見るログの出力
    RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "ALPHA: %f / %f", alpha_, alpha_threshold_);
    if(alpha < alpha_threshold_)
    {
        //! 膨張リセットを実行
        expansionReset();
        //! またよくわからん処理
        for(auto & p : particles_)
        {
            p.w_ *= p.likelihood(map_.get(), scan);
        }
    }

    //! リサンプリングするか重みづけをし直すかの処理
    //! normalize belief : [形容詞] + [名詞] 正規化する信念
    if(normalizeBelief() > 0.000001)
    {
        //! リサンプリング
        resampling();
    }
    else
    {
        //! 重みの初期化
        resetWeight();
    }
}
```

## Mcl::motionUpdate
## Mcl::meanPose
## Mcl::resampling
## Mcl::resetWeight

# amcl
## alpha の計算？
amcl では alpha が7つ用意されている？
`alpha1`, `alpha2`, `alpha3`, `alpha4`, `alpha5`, `alpha_slow`, `alpha_fast`
これらの変数が見られるのは `amcl_node.cpp` `motion_model/differential_motion_model.cpp`, `motion_model/omni_motion_model.cpp`, `pf/pf.c`

`amcl_node.cpp`
```
rcl_interfaces::msg::SetParametersResult AmclNode::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
    ~~~
    //! パラメータの取得
    for( ~~~ )
    {
        ~~~
        if( ~~~ )
        {
            if( ~~~ )
            {
                alpha1_ = parameter.as_double();
            }
            else if( ~~~ )
            {
                alpha2_ = parameter.as_double();
            }
            ...
        }
    }
}
```

```
AmclNode::on_configure(const rclcpp_lifecycle::State &)
{
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    initParameters();
    initTransforms();
    initParticleFilter();
    initLaserScan();
    initMessageFilters();
    initPubSub();
    initServices();
    initOdometry();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, get_node_base_interface());
    executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);
    return nav2_util::CallbackReturn::SUCCESS;
}
```

# 結果
## `pf_`
`emcl2_node.h`
``` C++
namespace emcl2
{
    class EMcl2Node : public rclcpp::Node
    {
        private:
            std::shared_ptr<ExpResetMcl2> pf_;
    }
}
```

## amcl と emcl2 共通の処理
### emcl2
- ExpResetMcl2::nonPenetrationRate
    alpha の計算をしている関数
- Mcl::motionUpdate
    odom の移動に合わせて particle を移動させる？
- Mcl::meanPose
    パーティクルの中心 (平均) を求めてるのかな？
- Mcl::resampling
    リサンプリング


# 時間計測プログラム
## 使い方
``` C++
#include "emcl2/TimeCounter.hpp"
auto timecounter = TimeCounter::TimeCounter(path, node_name_);
timecounter.startCounter();
timecounter.stopCounter();
```

