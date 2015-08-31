// Kinect2Grabber is pcl::Grabber to retrieve the point cloud data from Kinect v2 using Kinect for Windows SDK 2.x.
// This source code is licensed under the MIT license. Please see the License in License.txt.

#ifndef KINECT2_GRABBER
#define KINECT2_GRABBER

#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace pcl
{
	struct pcl::PointXYZ;
	struct pcl::PointXYZRGB;
	struct pcl::PointXYZI;
	template <typename T> class pcl::PointCloud;

	template<class Interface>
	inline void SafeRelease(Interface *& IRelease)
	{
		if (IRelease != NULL){
			IRelease->Release();
			IRelease = NULL;
		}
	}

	class Kinect2Grabber : public pcl::Grabber
	{
	public:
		Kinect2Grabber();
		virtual ~Kinect2Grabber() throw ();
		virtual void start();
		virtual void stop();
		virtual bool isRunning() const;
		virtual std::string getName() const;
		virtual float getFramesPerSecond() const;

		typedef void (signal_Kinect2_PointXYZ)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>&);
		typedef void (signal_Kinect2_PointXYZRGB)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>&);
		typedef void (signal_Kinect2_PointXYZI)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>&);

	protected:
		boost::signals2::signal<signal_Kinect2_PointXYZ>* signal_PointXYZ;
		boost::signals2::signal<signal_Kinect2_PointXYZRGB>* signal_PointXYZRGB;
		boost::signals2::signal<signal_Kinect2_PointXYZI>* signal_PointXYZI;

		pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ(UINT16* depthBuffer);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB(RGBQUAD* colorBuffer, UINT16* depthBuffer);
		pcl::PointCloud<pcl::PointXYZI>::Ptr convertRGBDepthToPointXYZI(RGBQUAD* colorBuffer, UINT16* depthBuffer);
		BYTE RGBToGreyScale(BYTE r, BYTE g, BYTE b);
		double sRBG_to_linear(double);
		double	linear_to_sRGB(double);

		boost::thread thread;
		mutable boost::mutex mutex;

		void threadFunction();

		bool quit;
		bool running;

		HRESULT result;
		IKinectSensor* sensor;
		ICoordinateMapper* mapper;
		IColorFrameSource* colorSource;
		IColorFrameReader* colorReader;
		IDepthFrameSource* depthSource;
		IDepthFrameReader* depthReader;

		int colorWidth;
		int colorHeight;
		std::vector<RGBQUAD> colorBuffer;

		int depthWidth;
		int depthHeight;
		std::vector<UINT16> depthBuffer;
	};
}
#endif KINECT2_GRABBER

