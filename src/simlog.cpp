#include "simlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"

//#include <QDateTime>
//#include <QDebug>
//#include <QString>


SimLog& SimLog::Instance()
{
	static SimLog log;
	return log;
}



void SimLog::InitSimLog(std::string logger_name, std::string file_name, bool broate,int log_level)
{
	//������־�ȼ�
	spdlog::set_level(static_cast<spdlog::level::level_enum>(log_level));
	//������־Ϊ�첽��־��������������־�ļ���һֱд��

	if(broate==false)
	my_logger_ = spdlog::basic_logger_mt<spdlog::async_factory >(logger_name, file_name);
	else
	my_logger_ = spdlog::rotating_logger_mt<spdlog::async_factory >(logger_name, file_name, 1024 * 1024 * 10, 100);
	;
	//���������󼶱����ϵ�����ˢ�µ���־
	my_logger_->flush_on(spdlog::level::err);
	//ÿ����ˢ��һ��
	spdlog::flush_every(std::chrono::seconds(3));

	//����
	for (int i = 0; i < 101; i++)
	{
		//my_logger_->info("SimLog::Async message #{}", i);
	}
}

void SimLog::EndLog()
{
	spdlog::shutdown();
}

SimLog::SimLog()
{

}

SimLog::~SimLog()
{
	EndLog();
}


void SimLog::SetLevel(int level)
{
	spdlog::set_level(static_cast<spdlog::level::level_enum>(level));
}


//SimLog::Instance().InitSimLog("test", "log.txt");
//int i = 10;
//double d_number = 10.01;
//LDebug("SimLog::Async message");
//LDebug("SimLog::Async message #{0},d_number:{1}", i, d_number);
