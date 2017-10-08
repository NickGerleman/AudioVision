#pragma once

class PeriodicTimer {
public:
	using TimerTask = std::function<void(void)>;

	PeriodicTimer(boost::asio::io_service& ioService,
		const std::chrono::microseconds& period,
		const TimerTask& task);

	void start(bool runOnStart = true);
	void stop();

private:
	void cbTimer(const boost::system::error_code& ec);

	boost::asio::io_service& ioService;
	boost::asio::steady_timer timer;

	std::chrono::microseconds period;

	TimerTask task;

	bool running;
};