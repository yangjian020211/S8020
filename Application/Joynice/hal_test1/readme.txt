
1、 黑盒子日志：ss_test_datalog.h/cpp
    现阶段测试了两个msg, 1 和 2,结果为2个正弦函数,其中1初始化阶段为一条斜线，进入循环后是正常正弦曲线;
    如需添加更多请修改ss_test_datalog.cpp 中_test_list[]数组成员

    多次写入的文件编号应该递增。
   
    cvt_log.py 用于log文件转码。控制台输入：python cvt_log.py ./raw_log_file.log ./output.txt    
    请修改 MSG_ID = b'\x01' or b'\x02' or ... 指定转换消息的ID，因为一次只能转一条。
    如果是IDE直接运行python，可以直接改代码指定消息ID、输入和输出文件名。如：
    MSG_ID = b'\x01'
    bin_file_name = './datafile_00000002.log' 
    out_file_name = './out.txt'  			
	   

2、 data_persist: ss_test_datalog.h/cpp
    需要反复开机关机测试，多次均无错误。
   