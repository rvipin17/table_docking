#include <gtest/gtest.h>
#include <monitoring_core/monitor.h>
#include <iostream>



TEST(MonitoringCore, addValueString)
{
    Monitor monitor_1;

    for(int i=0;i<500;i++)
    {
        std::string Test_key(i, 'a');
        std::string Test_unit(i, 'b');
        std::string Test_value(i, 'c');

        monitor_1.addValue(Test_key, Test_value, Test_unit,0.0);
        ASSERT_EQ(Test_key, monitor_1.ma.info[0].values[i].key);
        ASSERT_EQ(Test_unit, monitor_1.ma.info[0].values[i].unit);
        ASSERT_EQ(Test_value, monitor_1.ma.info[0].values[i].value);
    }
}

TEST(MonitoringCore, addValueFloatRand)
{

    Monitor monitor_2;
    Monitor monitor_3;
    Monitor monitor_4;

    monitor_2.addValue("Float_Test",3.402823466e+38f, "Float_Test",0.0);
    monitor_3.addValue("Float_Test",-3.402823466e+38f, "Float_Test",0.0);
    monitor_4.addValue("Float_Test",0, "Float_Test",0.0);

    ASSERT_LE("340280346638528859811704183484516925440.000000", monitor_2.ma.info[0].values[0].value);
    ASSERT_GE("-340284346638528859811704183484516925440.000000", monitor_3.ma.info[0].values[0].value);
    ASSERT_GE("340284306638528859811704183484516925440.000000", monitor_2.ma.info[0].values[0].value);
    ASSERT_LE("-340280346638528859811704183484516925440.000000", monitor_3.ma.info[0].values[0].value);
    ASSERT_EQ("0.000000", monitor_4.ma.info[0].values[0].value);
}

TEST(MonitoringCore, addValueFloat)
{
    Monitor monitor_5;
    Monitor monitor_6;
    float RandemNumber1;
    float RandemNumber2;
    for(int i=0;i<1000;i++)
    {
        float x=123.123*i;
        monitor_5.addValue("Float_Test",x, "Float_Test",0.0);
        monitor_6.addValue("Float_Test",-x, "Float_Test",0.0);
        RandemNumber1= atof(monitor_5.ma.info[0].values[0].value.c_str());
        RandemNumber2= atof(monitor_6.ma.info[0].values[0].value.c_str());

        ASSERT_EQ(RandemNumber1,x);
        ASSERT_EQ(RandemNumber2,-x);
    }

}

TEST(MonitoringCore, addValueErrorLevel)
{

    Monitor monitor_7;

    for(int i=0;i<=4;i++)
    {
        float y=0.0;
        ++y;
        monitor_7.addValue("ErrorLevel_Test",0.0, "ErrorLevel_Test",y);
        ASSERT_EQ(monitor_7.ma.info[0].values[0].errorlevel,y);
    }
}

TEST(MonitoringCore, aggregationLast)
{
    Monitor monitor;

    for(int i=0;i<=4;i++)
    {
        monitor.addValue("test_key", i, "", 0.0, AggregationStrategies::LAST);
    }
    ASSERT_EQ(monitor.ma.info[0].values.size(),1);
    ASSERT_EQ(monitor.ma.info[0].values[0].key, "test_key");
    ASSERT_EQ(atof(monitor.ma.info[0].values[0].value.c_str()), 4);
}

TEST(MonitoringCore, aggregationFirst)
{
    Monitor monitor;

    for(int i=0;i<=4;i++)
    {
        monitor.addValue("test_key", i, "", 0.0, AggregationStrategies::FIRST);
    }
    ASSERT_EQ(monitor.ma.info[0].values.size(),1);
    ASSERT_EQ(monitor.ma.info[0].values[0].key, "test_key");
    ASSERT_EQ(atof(monitor.ma.info[0].values[0].value.c_str()), 0);
}

TEST(MonitoringCore, aggregationMin)
{
    Monitor monitor;

    monitor.addValue("test_key", 13, "", 0.0, AggregationStrategies::MIN);
    monitor.addValue("test_key", 30, "", 0.0, AggregationStrategies::MIN);
    monitor.addValue("test_key", 12, "", 0.0, AggregationStrategies::MIN);
    monitor.addValue("test_key", 14, "", 0.0, AggregationStrategies::MIN);

    ASSERT_EQ(monitor.ma.info[0].values.size(),1);
    ASSERT_EQ(monitor.ma.info[0].values[0].key, "test_key");
    ASSERT_EQ(atof(monitor.ma.info[0].values[0].value.c_str()), 12);
}

TEST(MonitoringCore, aggregationMax)
{
    Monitor monitor;


    monitor.addValue("test_key", 0, "", 0.0, AggregationStrategies::MAX);
    monitor.addValue("test_key", 30, "", 0.0, AggregationStrategies::MAX);
    monitor.addValue("test_key", 12, "", 0.0, AggregationStrategies::MAX);

    ASSERT_EQ(monitor.ma.info[0].values.size(),1);
    ASSERT_EQ(monitor.ma.info[0].values[0].key, "test_key");
    ASSERT_EQ(atof(monitor.ma.info[0].values[0].value.c_str()), 30);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
