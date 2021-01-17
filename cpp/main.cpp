#include "hedder.h";

class MouseEvent
{
public:
	void change(int x, int y, int eventType)
	{
		_x = x;
		_y = y;
		_eventType = eventType;
	}
	int getEventType()
	{
		return _eventType;
	}
	int getX()
	{
		return _x;
	}
	int getY()
	{
		return _y;
	}

private:
	int _x, _y;		// click point
	int _eventType; // mouse event type
};

// mouse callback
void CallBackFunction(int eventType, int x, int y, int flags, void* userdata)
{
	MouseEvent* m = static_cast<MouseEvent*>(userdata);
	m->change(x, y, eventType);
}

// digit aligment
void DigitAligment(int val, int num = 3, bool endl = false)
{
	std::cout << std::setw(num) << std::setfill(' ') << val;
	if (!endl)
		printf("\n");
}
void DigitAligment(float val, bool endl = false)
{
	std::cout << std::fixed;
	std::cout << std::setw(11) << std::setfill(' ') << std::setprecision(6) << val;
	if (endl)
		printf("\n");
}
#pragma region ClassSection

class DepthSensor
{
public:
	DepthSensor()
	{
		openni::OpenNI::initialize();
	}
	~DepthSensor()
	{
		cv::destroyAllWindows();
		colorStream.destroy();
		depthStream.destroy();
		device.close();
		openni::OpenNI::shutdown();
	}
	/// <summary>
	/// initialize depth sensor
	/// </summary>
	void depthInitialize()
	{
		// open depth sensor
		if (device.open(openni::ANY_DEVICE) != openni::STATUS_OK) {
			throw std::runtime_error("openni::Device::open() failed.");
		}
		// create color stream
		colorStream.create(device, openni::SENSOR_COLOR);
		// create depth stream
		depthStream.create(device, openni::SENSOR_DEPTH);

		changeResolutioin(colorStream);
		changeResolutioin(depthStream);

		// change IMAGE_REGISTRATION_DEPTH_TO_COLOR mode
		if (device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
			device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		}

		// start stream
		colorStream.start();
		depthStream.start();
	}

	void updateColorFrame()
	{
		// read color frame
		colorStream.readFrame(&colorFrame);
		showColorStream(colorFrame, colorImage);
		lineDesc(colorImage);

	}

	void updateDepthFrame()
	{
		// read depth frame
		depthStream.readFrame(&depthFrame);
		showDepthStream(depthFrame, depthImage);
	}


	openni::DepthPixel getPointDistance(int pixelX, int pixelY)
	{
		return (openni::DepthPixel)showPointDistance(depthFrame, pixelX, pixelY);
	}

	void getDepthToWorld(int pixelX, int pixelY, float& x, float& y, float& z)
	{
		openni::DepthPixel pixelZ = getPointDistance(pixelX, pixelY);

		openni::CoordinateConverter::convertDepthToWorld(depthStream, pixelX, pixelY, pixelZ, &x, &y, &z);
	}

private:
	void changeResolutioin(openni::VideoStream& stream)
	{
		openni::VideoMode mode = stream.getVideoMode();
		mode.setFps(30);
		mode.setResolution(320, 240);
		stream.setVideoMode(mode);
	}
	/// <summary>
	/// カラー画像をOpenCVで表示できるように変換
	/// </summary>
	void showColorStream(const openni::VideoFrameRef colorFrame, cv::Mat& colorImage)
	{
		// OpenCV�̌`�ɂ���
		colorImage = cv::Mat(colorFrame.getHeight(), colorFrame.getWidth(),
			CV_8UC3, (unsigned char*)colorFrame.getData());

		// BGR��RGB�ɕϊ�����
		cv::cvtColor(colorImage, colorImage, cv::COLOR_BGR2RGB);

	}

	/// <summary>
	/// Depth画像をOpenCVで表示できるように変換
	/// </summary>
	void showDepthStream(const openni::VideoFrameRef& depthFrame, cv::Mat& depthImage)
	{
		// �����f�[�^���摜������(16bits)
		depthImage = cv::Mat(depthFrame.getHeight(),
			depthFrame.getWidth(), CV_16UC1, ((unsigned short*)depthFrame.getData()));

		// 0~10000mm�܂ł̃f�[�^��0~255�ɐ��K��
		depthImage.convertTo(depthImage, CV_8U, 255.0 / 10000);
		lineDesc(depthImage);
		cv::imshow("Depth Frame", depthImage);
	}

	/// <summary>
	/// クリック座標の深度を取得
	/// </summary>
	unsigned short showPointDistance(const openni::VideoFrameRef& depthFrame, int pixelX, int pixelY)
	{
		openni::VideoMode videoMode = depthStream.getVideoMode();

		int centerX = pixelX;
		int centerY = pixelY;
		int centerIndex = (centerY * videoMode.getResolutionX()) + centerX;

		unsigned short* depth = (unsigned short*)depthFrame.getData();

		return depth[centerIndex];
	}

	void lineDesc(cv::Mat& src)
	{
		cv::line(src, cv::Point(0, src.rows / 2),
			cv::Point(src.cols, src.rows / 2), cv::Scalar(255, 255, 255), 1);
		cv::line(src, cv::Point(src.cols / 2, 0),
			cv::Point(src.cols / 2, src.rows), cv::Scalar(255, 255, 255), 1);
	}

	openni::Device device;              // センサー情報が格納される
	openni::VideoStream colorStream;    // カラー
	openni::VideoStream depthStream;    // Depth
	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef colorFrame;
public:
	cv::Mat colorImage;                 // カラー画像
	cv::Mat depthImage;                 // Depth画像
};

#pragma endregion

void measureF(std::vector<std::vector<float>> res, std::string c, float wz)
{
	float g = 0;
	int i = 0;
	struct tm stm;
	time_t tim;
	char s[100];

	if (res.size() > 1) {
		//- 結果をファイル格納する準備 ----------------------
		tim = time(NULL);
		localtime_s(&stm, &tim);
		int status = _mkdir(".\\results");
		strftime(s, 100, "results\\%Y%m%d%H%M%S_", &stm);
		std::string tale = c + ".txt";
		strcat_s(s, tale.c_str());
		std::ofstream ofs(s);
		//-------------------------------------------------
		ofs << "RESULT " + c << std::endl;
		ofs << "World_Depth(0,0): " << std::to_string(wz) << std::endl << std::endl;
		res.erase(res.begin());
		for (const auto& vect : res)
		{
			ofs << " 長さ    : " << vect[0] << " [mm]" << std::endl;
			ofs << "     誤差: " << vect[1] << " [mm]" << std::endl;
			g += vect[1];
			i++;

		}
		if (i != 0)
			g = g / i;
		ofs << "\n誤差平均: " << g << " [mm]\n試行回数:" << i << " [��]\n\n";
		if (!status)
			std::cout << "mkdir results.\n";
		std::cout << "Wrote results in " << s << std::endl;
	}
}

//-------------------------------------------------------------------------------------------------
// Main
int main(int argc, const char* argv[])
{
	cv::String winName = "ColorStream";
	cv::Mat inputImage, hsvImage;
	MouseEvent mEvnt;
	std::vector<std::vector<float>> res_x = {};
	std::vector<std::vector<float>> res_y = {};

	bool isLeftDown = false;
	try {
		DepthSensor sensor;
		sensor.depthInitialize();
		float bX, bY, bZ;
		bX = bY = bZ = 0.0;
		std::string c;
		while (true) {
			std::cout
				<< "------------------------------------------------\n"
				<< "w: width between two points\n"
				<< "h: height between two points\n"
				<< "x: x from world origin\n"
				<< "y: y from world origin\n"
				<< "q: exit the program\n"
				<< "------------------------------------------------\n";
			std::cout 
				<< "Type [w], [h], [x], [y] or [q] and [Enter].\n$ ";
			std::cin >> c;
			if (c.compare("x") && c.compare("y") && c.compare("h") && c.compare("w") && c.compare("q"))
				std::cout << c + " is not {w, h, x, y, q}\n\n";
			else if (!c.compare("q"))
				exit(0);
			else
				break;
		}

		while (1) {
			sensor.updateDepthFrame();
			sensor.updateColorFrame();
			inputImage = sensor.colorImage;
			cv::imshow(winName, inputImage);


			cv::setMouseCallback(winName, CallBackFunction, &mEvnt);
			const int key = cv::waitKey(1);
			float wx, wy, wz;


			if (mEvnt.getEventType() == cv::EVENT_LBUTTONDOWN)
			{
				if (!isLeftDown)
				{
					int x = mEvnt.getX(), y = mEvnt.getY();					  // クリック座標
					int ocx = y, ocy = x;									  // OpenCV座標系

					// クリックされたピクセル
					std::cout << "Pixel : [ ";
					DigitAligment(x, 4, 1);
					std::cout << ", ";
					DigitAligment(y, 4, 1);
					std::cout << " ]" << std::endl;

					sensor.getDepthToWorld(x, y, wx, wy, wz);

					std::cout << "World-------------------------" << std::endl;
					std::cout << " X   : ";
					DigitAligment(wx, false);
					printf(" [mm]\n");
					std::cout << " Y   : ";
					DigitAligment(wy, false);
					printf(" [mm]\n");
					std::cout << " Z   : ";
					DigitAligment(wz, false);
					printf(" [mm]\n");
					std::cout << "------------------------------\n" << std::endl;

					if (!c.compare("w") || !c.compare("h")) {
						std::cout << "From: [ ";
						DigitAligment(bX, false);
						std::cout << ", ";
						DigitAligment(bY, false);
						std::cout << ", ";
						DigitAligment(bZ, false);
						std::cout << " ]\nTo  : [ ";
						DigitAligment(wx, false);
						std::cout << ", ";
						DigitAligment(wy, false);
						std::cout << ", ";
						DigitAligment(wz, false);
						std::cout << " ]\n";
					}


					if (!c.compare("w")) {
						float measure_x = fabsf(wx - bX);
						res_x.push_back({ measure_x, fabsf(measure_x - CORRECT_X) });
						std::cout << " Width: " << measure_x << "[mm]" << std::endl << std::endl;
						std::cout << "Clicked " << res_x.size() << " times.\n";
					}
					else if (!c.compare("h")) {
						float measure_y = fabsf(wy - bY);
						res_y.push_back({ measure_y, fabsf(measure_y - CORRECT_Y) });
						std::cout << " Height: " << measure_y << "[mm]" << std::endl << std::endl;
						std::cout << "Clicked " << res_y.size() << " times.\n";
					}
					else if (!c.compare("x")) {
						float measure_x = fabsf(wx);
						res_x.push_back({ measure_x, fabsf(measure_x - CORRECT_WX) });
						std::cout << " WX: " << measure_x << "[mm]" << std::endl << std::endl;
						std::cout << "Clicked " << res_x.size() << " times.\n";
					}
					else if (!c.compare("y")) {
						float measure_y = fabsf(wy);
						res_y.push_back({ measure_y, fabsf(measure_y - CORRECT_WY) });
						std::cout << " WY: " << measure_y << "[mm]" << std::endl << std::endl;
						std::cout << "Clicked " << res_y.size() << " times.\n";
					}

					bX = wx;
					bY = wy;
					bZ = wz;

					isLeftDown = true;
				}
			}
			// qが押された処理
			else if (key == 'q')
			{
				sensor.getDepthToWorld(sensor.colorImage.cols / 2, sensor.colorImage.rows / 2, wx, wy, wz);
				if (!c.compare("x") || !c.compare("w"))
					measureF(res_x, c, wz);
				else if (!c.compare("y") || !c.compare("h"))
					measureF(res_y, c, wz);

				std::cout << "\n\n  Please Press Any Key in " + winName
					<< std::endl << std::endl;
				cv::waitKey(0);
				break;
			}
			else
			{
				isLeftDown = false;
			}
		}
	}
	catch (std::exception&) {
		std::cout << openni::OpenNI::getExtendedError() << std::endl;
	}

	return 0;
}