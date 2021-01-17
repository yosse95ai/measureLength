#include "hedder.h";

class MouseEvent
{
public:
	// �}�E�X�ʒu���X�V
	void change(int x, int y, int eventType)
	{
		_x = x;
		_y = y;
		_eventType = eventType;
	}
	// �}�E�X�C�x���g�̃^�C�v���擾����
	int getEventType()
	{
		return _eventType;
	}
	// �}�E�X��x���W
	int getX()
	{
		return _x;
	}
	// �}�E�X��y���W
	int getY()
	{
		return _y;
	}

private:
	int _x, _y;		// �}�E�X���W( x, y )
	int _eventType; // �}�E�X�C�x���g�̃^�C�v
};

// �R�[���o�b�N�֐�
void CallBackFunction(int eventType, int x, int y, int flags, void* userdata)
{
	MouseEvent* m = static_cast<MouseEvent*>(userdata);
	m->change(x, y, eventType);
}

// ���l�̌������낦�ăR���\�[���o��
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
	/// �Z���T�[�̏������֐�
	/// </summary>
	void depthInitialize()
	{
		// �f�o�C�X���擾
		if (device.open(openni::ANY_DEVICE) != openni::STATUS_OK) {
			throw std::runtime_error("openni::Device::open() failed.");
		}
		// �J���[�X�g���[����L���ɂ���
		colorStream.create(device, openni::SENSOR_COLOR);
		// Depth�X�g���[����L���ɂ���
		depthStream.create(device, openni::SENSOR_DEPTH);

		changeResolutioin(colorStream);
		changeResolutioin(depthStream);

		// �f�o�C�X��IMAGE_REGISTRATION_DEPTH_TO_COLOR�ɑΉ����Ă��邩
		if (device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
			// �Ή����Ă���Ȃ�Z�b�g����
			device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		}

		// �X�g���[���̊J�n
		colorStream.start();
		depthStream.start();
	}

	void updateColorFrame()
	{
		// �X�V���ꂽ�t���[�����擾����
		colorStream.readFrame(&colorFrame);
		// �t���[���f�[�^��\���ł���`�ɕϊ�����
		showColorStream(colorFrame, colorImage);
		lineDesc(colorImage);

	}

	void updateDepthFrame()
	{
		// �X�V���ꂽ�t���[�����擾����
		depthStream.readFrame(&depthFrame);
		// �t���[���f�[�^��\���ł���`�ɕϊ�����
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
	/// �J���[�X�g���[����OpenCV�ŕ\���ł���`�ɂ���
	/// </summary>
	/// <param name="colorFrame"></param>
	/// <returns>cv::Mat�^�̃t���[��</returns>
	void showColorStream(const openni::VideoFrameRef colorFrame, cv::Mat& colorImage)
	{
		// OpenCV�̌`�ɂ���
		colorImage = cv::Mat(colorFrame.getHeight(), colorFrame.getWidth(),
			CV_8UC3, (unsigned char*)colorFrame.getData());

		// BGR��RGB�ɕϊ�����
		cv::cvtColor(colorImage, colorImage, cv::COLOR_BGR2RGB);

	}

	/// <summary>
	/// Depth�X�g���[����\���ł���`���ɕϊ�
	/// </summary>
	/// <param name="depthFrame"></param>
	/// <param name="depthImage"></param>
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
	/// ���W�̉��s�����߂�
	/// </summary>
	/// <param name="depthImage"></param>
	/// <param name="depthFrame"></param>
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

	openni::Device device;              // �g�p����f�o�C�X
	openni::VideoStream colorStream;    // �J���[�X�g���[��
	openni::VideoStream depthStream;    //  Depth�X�g���[��
	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef colorFrame;
public:
	cv::Mat colorImage;                 // �\���pDepth�f�[�^
	cv::Mat depthImage;                 // �\���pColor�f�[�^
};

#pragma endregion

void measureF(std::vector<std::vector<float>> res, std::string c, float wz)
{
	float g = 0;
	int i = 0;

	// �t�@�C�����̕ϐ���p��
	struct tm stm;
	time_t tim;
	char s[100];

	if (res.size() > 1) {
		//- �t�@�C���o�͏��� ------------------------------
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
			ofs << " ����    : " << vect[0] << " [mm]" << std::endl;
			ofs << "     �덷: " << vect[1] << " [mm]" << std::endl;
			g += vect[1];
			i++;

		}
		if (i != 0)
			g = g / i;
		ofs << "\n�덷����: " << g << " [mm]\n���s��:" << i << " [��]\n\n";
		if (!status)
			std::cout << "mkdir results.\n";
		std::cout << "Wrote results in " << s << std::endl;
	}
}

//-------------------------------------------------------------------------------------------------
// Main�֐�
int main(int argc, const char* argv[])
{
	cv::String winName = "ColorStream";
	cv::Mat inputImage, hsvImage;
	MouseEvent mEvnt;
	std::vector<std::vector<float>> res_x = {};
	std::vector<std::vector<float>> res_y = {};

	// ���N���b�N�C�x���g�̃t���O
	bool isLeftDown = false;
	try {

		// �Z���T�[������������
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

		// ���C�����[�v
		while (1) {
			sensor.updateDepthFrame();
			sensor.updateColorFrame();
			inputImage = sensor.colorImage;
			cv::imshow(winName, inputImage);


			// �}�E�X�R�[���o�b�N�̐ݒ�
			cv::setMouseCallback(winName, CallBackFunction, &mEvnt);
			// �L�[���͂��擾
			const int key = cv::waitKey(1);
			float wx, wy, wz;



			// ���N���b�N����������\��
			if (mEvnt.getEventType() == cv::EVENT_LBUTTONDOWN)
			{
				// �N���b�N������ɑ΂��ďo�͂���
				if (!isLeftDown)
				{
					int x = mEvnt.getX(), y = mEvnt.getY();					  // �}�E�X��xy���W�擾
					int ocx = y, ocy = x;									  // OpenCV�p�ɍ��W�ϊ�

					// �N���b�N��̃}�E�X�̍��W���o��
					std::cout << "Pixel : [ ";
					DigitAligment(x, 4, 1);
					std::cout << ", ";
					DigitAligment(y, 4, 1);
					std::cout << " ]" << std::endl;

					sensor.getDepthToWorld(x, y, wx, wy, wz);

					// ���[���h���W�n���o��
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
			// q�L�[�������ꂽ��I��
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
			// �����Ȃ���΍��N���b�N�C�x���g�̃t���O��������
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