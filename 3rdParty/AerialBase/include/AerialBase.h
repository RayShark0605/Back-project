
#ifndef __AERIABASE_H__
#define __AERIABASE_H__

#include <vector>

#ifdef AERIABASE_EXPORTS
    #define AERIABASE_API __declspec(dllexport)
	#include <xercesc/dom/DOM.hpp>
	XERCES_CPP_NAMESPACE_USE
#else
    #define AERIABASE_API __declspec(dllimport)
    #pragma comment(lib, "SVSAerialProject.lib")
    #pragma message("Automatically Linking With SVSAerialProject.dll")
//#define DOMNode  void
#endif //AERIABASE_EXPORTS



#pragma warning(disable:4251 4996)



class AERIABASE_API CAerialBaseObject
{
public:
	virtual bool SaveXML(void* lpNode) = 0;
	virtual bool FromXML(void* lpNode) = 0;
};

class AERIABASE_API CCamera : public CAerialBaseObject
{
public:
	enum SENSORTYPE { SENSOR_FRAME = 0, SENSOR_CCD };

	enum MOUNTTYPE { MNT_0 = 0, MNT_90, MNT_180, MNT_270};

	struct DISTORTIONITEM
	{
		double len;
		double distortion;
	};

	union DISTORTION
	{
		struct Coefficients
		{
			double K[8];
			double P[4];
			double Alpha;
			double Belta;
		}COEFFS;

		struct Table
		{
			int   nItem;
			DISTORTIONITEM* Items;
		}TABLE;

		struct Ebner_12
		{
			double V[12];
		}EBNER;

		struct Agrun_44
		{
			double V[44];
		}AGRUN;
	};


	CCamera();
	CCamera(CCamera* other);
	virtual ~CCamera();

// 	bool        LoadFromFile(const char* lpszCamFile);
// 	bool        SaveToFile(const char* lpszCamFile);

	/************************************************************************/
	/* 设置和获取相机ID，系统利用这个相机ID来唯一标识一个相机               */
	/************************************************************************/
	void        SetID(const char* lpszID);
	const char* GetID() const;
	/************************************************************************/
	/* 设置和获取相机的序列号，可选项                                       */
	/************************************************************************/
	void        SetSerialNumber(char* lpszSerialNum);
	const char* GetSerialNumber() const;
	/************************************************************************/
	/* 设置和获取相机的品牌                                                 */
	/************************************************************************/
	void        SetBrand(char* lpszBrand);
	const char* GetBrand() const;

	/************************************************************************/
	/* 设置和获取相机的评论                                                 */
	/************************************************************************/
	void        SetComment(char* lpszComment);
	const char* GetComment() const;

	const char* GetDateTime() const;

	/************************************************************************/
	/* 设置和获取相机的焦距                                                 */
	/************************************************************************/
	void        SetFocalLength(double focalLength );
	double      GetFocalLength() const;


	void        SetPPS(double x, double y);
	void        GetPPS(double&x, double& y);

	void        SetPPA(double x, double y);
	void        GetPPA(double&x, double& y);

	void        SetGNSSAntennaOffset(double X, double Y, double Z);
	void        GetGNSSAntennaOffset(double&X, double&Y, double&Z);

	void        SetMountType(MOUNTTYPE type);
	MOUNTTYPE   GetMountType();


	/************************************************************************/
	/* 设置和获取相机畸变参数                                               */
	/************************************************************************/
	/* 0 表示没有 1 表示系数, 2表示表 11表示计算机视觉畸变系数  12表示Ebner12参数 44 表示Agrun44参数*/
	int                GetDistortionType() const; 
	void               SetDistortion(int type, DISTORTION* lpDistortion);	
	DISTORTION*        GetDistortion() const;

	/************************************************************************/
	/* 设置和获取相机类型：框幅相机或数码                                   */
	/************************************************************************/
	virtual SENSORTYPE GetSensorType( ) const = 0;

	//implementation of CAeriaBaseObject
	virtual bool SaveXML(void* lpNode);
	virtual bool FromXML(void* lpNode);

private:
	char            m_szTime[64];
	char            m_szID[64];
	char            m_szSerialNumber[64];
	char            m_szBrand[32];
	char            m_szComment[128];
	
	double          m_GNSSAntennaOffset[3];
	MOUNTTYPE       m_MNTType;

	double          m_FocalLength;
	int             m_DistortionType;
	DISTORTION*     m_lpDistortion;

	double          m_PPS[2];
	double          m_PPA[2];
};

class AERIABASE_API CFrameCamera : public CCamera
{
public:
	enum   FUDICALMARKORIGIN { PPS = 0, PPA, FC };

	struct FUDICALMARK
	{
		int id;
		double x, y;
	};

	CFrameCamera();
	CFrameCamera(CFrameCamera* lpOther);

	virtual ~CFrameCamera();

    void               SetFudicalMarkOrigin( FUDICALMARKORIGIN mark_orig );
	FUDICALMARKORIGIN  GetFudicalMarkOrigin() const;

	int                GetNumFudicalMark() const;
	int                AddFudicalMark(const FUDICALMARK* lpMark);
	FUDICALMARK*       GetFudicalMarkByIndex(int index);
	FUDICALMARK*       GetFudicalMarkByID(int id);  
	void               ClearFudicalMarks();

	//implementation of CCamera
	virtual SENSORTYPE GetSensorType( ) const;

	//implementation of CAeriaBaseObject
	virtual bool SaveXML(void* lpNode);
	virtual bool FromXML(void* lpNode);

private:
	FUDICALMARKORIGIN   m_FudicalMarkOrig; //框标坐标是相对于PPS，PPA或是FC
	std::vector<FUDICALMARK> m_FudicalMarks;
};

class AERIABASE_API CCCDCamera : public CCamera
{
public:
	//实现摄影测量像方空间畸变模型到计算机视觉物方空间畸变模型转换
	static bool PhotogrammetryCTComputerVision(CCCDCamera* lpPhtCam, CCCDCamera* lpCVCam, bool bCalcSkew, double& rms, double& maxvv);
	//实现计算机视觉物方空间畸变模型到摄影测量像方空间畸变模型转换
	static bool ComputerVisionCTPhotogrammetry(CCCDCamera* lpCVCam, CCCDCamera* lpPhtCam, double& rms, double& maxvv);


	CCCDCamera();
	CCCDCamera(CCCDCamera* other);
	virtual ~CCCDCamera();


	void  SetSensorSizeInPixel( int width, int height);
	void  GetSensorSizeInPixel( int& width, int& height);

	void  SetSensorPixelSize(double x, double y);
	void  GetSensorPixelSize(double& x, double& y);

	void  ProvidePPA(bool bEnable);
	bool  IsProvidePPA() const;

	void  ProvidePPS(bool bEnable);
	bool  IsProvidePPS() const;

	//implementation of CCamera
	virtual SENSORTYPE GetSensorType( ) const;

	//implementation of CAeriaBaseObject
	virtual bool SaveXML(void* lpNode);
	virtual bool FromXML(void* lpNode);

private:	
	bool                       m_bProvidePPA;
	bool                       m_bProvidePPS;
	int                        m_nSensorSizeInPixel[2];
	double                     m_SensorPixelSize[2];

};


class CAerialBlock;
class CAerialStrip;
class CGPSIMUDatum;

class AERIABASE_API CAerialPhoto : public CAerialBaseObject
{
	friend class CAerialBlock;
public:

	struct  INNERORI
	{
		int  Type; //类型（0，仿射变换，1,2D透视变换

		union MODEL
		{
			//
			struct AFFINE
			{
				//实现从扫描坐标(ix, iy)到像坐标(px, py)的转换
				//px = a00 + a01 * ix + a02 * iy
				//py = b00 + b01 * ix + b02 * iy
				double a00, a01, a02;
				double b00, b01, b02;

				//实现从像坐标（px, py)到扫描坐标(ix, iy)的转换
				//ix = a10 + a11 * px + a12 * py
				//iy = b10 + b11 * px + b12 * py
				double a10, a11, a12;
				double b10, b11, b12;
			}Affine;

			struct PERSPECTIVE  
			{
				//实现从扫描坐标(ix, iy)到像坐标(px, py)的转换
				//K = e00 * ix + f00 * iy + 1.0;
				//X = a00 * ix + a01 * iy + a02;
				//Y = b00 * ix + b01 * iy + b02;
				//px = X/K;   py = Y / K;
				double a00, a01, a02;
				double b00, b01, b02;
				double e00, f00;

				//实现从像坐标（px, py)到扫描坐标(ix, iy)的转换
				//K = e10 * px + f10 * py + 1.0;
				//X = a10 * px + a11 * py + a12;
				//Y = b10 * px + b11 * py + b12;
				//ix = X/K;   iy = Y / K;
				double a10, a11, a12;
				double b10, b11, b12;
				double e10, f10;
			}Perspective;

		}InOri;
		
	};	

	struct EXTORI
	{
		double POS[3];
		double matrix[9]; //定义了像空间坐标系到像空间辅助坐标系的转换关系
	};

	enum SCANORIENTATION { NORMAL = 0, ROT_90, ROT_180, ROT_270, 
		                   MIRROR_NORMAL, MIRROR_ROT_90,MIRROR_ROT_180,MIRROR_ROT_270};


	void             SetBlock(CAerialBlock* lpBlock);
	CAerialBlock*    GetBlock();

	void             SetID(const char* id);
	const char*      GetID() const;

	void             SetURL(const char* lpszPhotoURL);
	const char*      GetURL() const;

	void             SetActive(bool bActive = true) { m_bActive = bActive; }
	bool             IsActive() { return m_bActive; }

	void             SetDimension(int width, int height);
	void             GetDimension(int&width, int&height);

	void             SetScanOrientation(SCANORIENTATION);
	SCANORIENTATION  GetScanOrientation() const;

	void             SetInOri(const INNERORI* lpInOri);
	const INNERORI*  GetInOri() const;

	void             SetExtOri(const EXTORI* lpExtOri);
	const EXTORI*    GetExtOri( ) const;

	const char*      GetCameraID();

	void             SetCamera(CCamera* lpCamera);
	const CCamera*   GetCamera() const;


	void             SetXYZ(double X, double Y, double Z);
	void             GetXYZ(double&X, double&Y, double&Z);

	void             SetPOK(double  phi, double omega, double kappa);
	void             GetPOK(double* phi, double* omega, double* kappa);	

	void             SetOPK(double omega, double phi, double kappa);
	void             GetOPK(double* omega, double* phi, double* kappa);
	

	void             SetGPSIMU(CGPSIMUDatum* lpGPSIMU);
	CGPSIMUDatum*    GetGPSIMU();
	

	void             SetTerrainHeight(float h);
	float            GetTerrainHeight() const;

	void             SetStrip(CAerialStrip* lpStrip);
	CAerialStrip*    GetStrip();

	void             SetData(void* lpData);
	void*            GetData() const;


	//扫描和像坐标系间的转换
	void             PixelCS2ImageCS(double ix, double iy, double& px, double& py);
	void             ImageCS2PixelCS(double px, double py, double& ix, double& iy);

	//物方和像方的转换
	void             PixelRayIntersect(double ix, double iy, double Z, double& X, double& Y);
	void             ImageRayIntersect(double px, double py, double Z, double& X, double& Y);
	void             ObjectCS2PixelCS(double X, double Y, double Z, double& ix, double& iy);
	void             ObjectCS2ImageCS(double X, double Y, double Z, double& px, double& py);


	//implementation of CAeriaBaseObject
	virtual bool SaveXML(void* lpNode);
	virtual bool FromXML(void* lpNode);


protected:
	CAerialPhoto();
	CAerialPhoto(const char* id, const char* lpszPhotoFilePath);
	virtual ~CAerialPhoto();

	virtual void IntOri();
 
	void             DistortionFreeToObservable(double fpx, double fpy, double& opx, double& opy );
	void             DistortionObservableToFree(double opx, double opy, double& fpx, double& fpy );

	void             CorrectEarthCurvature(double x, double y, double& dx, double& dy );
	void             CorrectAtmoRefraction(double x, double y, double& dx, double& dy );
protected:
	char                    m_szID[64];
	char                    m_szPhotoURL[256];

	INNERORI                m_InOri;
	EXTORI                  m_ExtOri;
	SCANORIENTATION         m_ScanOrientation;
	float                   m_TerrainHeight;

	CAerialBlock*           m_lpBlock;
	CCamera*                m_lpCamera;
	CGPSIMUDatum*           m_lpGPSIMU;

private:
	CAerialStrip*           m_lpStrip;
	int                     m_nWidth;
	int                     m_nHeight;
	void*                   m_lpData;
	bool                    m_bActive;

};

class AERIABASE_API CAerialStrip : public CAerialBaseObject
{
	friend class CAerialBlock;
public:
	/************************************************************************/
	/* 设置和获取航带序号                                                   */
	/************************************************************************/
	void              SetID(const char* lpszIndex );
	const char*       GetID() const;

	/************************************************************************/
	/* 往航带中增加航片对象                                                 */
	/************************************************************************/
	bool              AddPhoto(CAerialPhoto* lpPhoto);
	/************************************************************************/
	/* 从航带中删除指定航片                                                 */
	/************************************************************************/
	bool              DeletePhoto(CAerialPhoto* lpPhoto);
	bool              DeletePhoto(const char*  lpszID);
	bool              DeletePhoto( int index );
    void              DeleteAllPhotos();
	/************************************************************************/
	/* 往航带中插入航片对象                                                 */
	/************************************************************************/
	bool              InsertPhoto(int index, CAerialPhoto* lpPhoto);
	/************************************************************************/
	/* 按照航片ID的升或降序排列航带中的航片对象                             */
	/************************************************************************/
	void              SortByPhotoID(bool bAscend = true);
	/************************************************************************/
	/* 使得航带中的航片对象按照现有的反排排列                               */
	/************************************************************************/
	void              Reverse();
	/************************************************************************/
	/* 获取航带中航片得个数                                                 */
	/************************************************************************/
	int               GetNumPhoto() const;
	/************************************************************************/
	/* 按照指定索引号从航带中提取相应的航片对象                             */
	/************************************************************************/
	CAerialPhoto*      GetPhoto(int index );
	/************************************************************************/
	/* 利用相片的ID从航带中查找航片对象                                     */
	/************************************************************************/
	CAerialPhoto*      GetPhoto(const char* lpszID);


	//implementation of CAeriaBaseObject
	virtual bool SaveXML(void* lpNode);
	virtual bool FromXML(void* lpNode);

protected:
	CAerialStrip();
	CAerialStrip(const char* lpszIndex);
	virtual ~CAerialStrip();

	/************************************************************************/
	/* 判断是否航带中已经存在指定的航片对象，存在，返回TRUE，否则FALSE      */
	/************************************************************************/
	bool              IsAlreadExist(CAerialPhoto* lpPhoto);

private:
	CAerialBlock*              m_lpAerialBlock;
	char                       m_szIndex[64];
	std::vector<CAerialPhoto*> m_PhotoList;
};


class AERIABASE_API CCameraXMLFile  : public CAerialBaseObject
{
public:
	CCameraXMLFile();
	~CCameraXMLFile();

	//implementation of CAeriaBaseObject
	virtual bool SaveXML(void* lpNode);
	virtual bool FromXML(void* lpNode);

//	bool        LoadFromFile(const char* lpszCamFile);
//	bool        SaveToFile(const char* lpszCamFile);

	bool        AddCam(CCamera* lpCamera);
	bool        DelCam(int index);
	bool        DelCam(const char* lpszCamID);

	int         GetCamNum() const;
	CCamera*    GetCam(int index);
	CCamera*    GetCam(const char* lpszCamID);

	void        Reset();

private:
	std::vector<CCamera*> m_CamList;
};

class AERIABASE_API CGPSIMUDatum : public CAerialBaseObject
{
public:
	CGPSIMUDatum();
	CGPSIMUDatum(const char* lpszID);
	CGPSIMUDatum(const char* lpszID, double X, double Y, double Z);
	CGPSIMUDatum(const char* lpszID, double X, double Y, double Z, double Omega, double Phi, double Kappa);
	~CGPSIMUDatum();

	void             SetID(const char* id);
	const char*      GetID() const;

	void  ActiveGPS(bool bActive = true);
	bool  IsActiveGPS() const;
	void  SetGPS(double X, double Y, double Z);
	void  GetGPS(double&X, double&Y, double&Z);
	

	void  ActiveIMU(bool bActive = true);
	bool  IsActiveIMU() const;
	void  SetIMU(double  Omega, double Phi, double Kappa);
	void  GetIMU(double& Omega, double&Phi, double& Kappa);

	CGPSIMUDatum& operator = (CGPSIMUDatum& Other);

	//implementation of CAeriaBaseObject
	virtual bool SaveXML(void* lpNode);
	virtual bool FromXML(void* lpNode);

protected:
	char   m_szID[64];
	bool   m_bGPSActive;
	double m_GPS[3];

	bool   m_bIMUActive;
	double m_IMU[3];

};

class AERIABASE_API CGCPDatum : CAerialBaseObject
{
public:
    CGCPDatum();
    CGCPDatum(const char* lpszID);
    CGCPDatum(const char* lpszID, double X, double Y, double Z);

    void             SetID(const char* id);
    const char*      GetID() const;

    void             SetXYZ(double X, double Y, double Z);
    void             GetXYZ(double&X, double&Y, double&Z);

    bool             IsHGCP();
    bool             IsVGCP();

     //设置控制点平面分组号
    void             SetXYStdDevClass(int XYClass);
    int              GetXYStdDevClass();

     //设置控制点高程分组号
    void             SetZStdDevClass(int ZClass);
    int              GetZStdDevClass();

    CGCPDatum& operator = (CGCPDatum& Other);

    //implementation of CAeriaBaseObject
    virtual bool SaveXML(void* lpNode);
    virtual bool FromXML(void* lpNode);

protected:
    char   m_szID[64];

    double m_XYZ[3];

    int    m_XYStdDevClass; /* 1-9控制分组 10检查点 0弃用*/
    int    m_ZStdDevClass;  /* 1-9控制分组 10检查点 0弃用*/

};

class AERIABASE_API IAerialProgress
{
public:
	virtual void SetRange(unsigned int range) = 0;
	virtual void SetPos(unsigned int pos) = 0;
	virtual void StepIt() = 0;
};


class AERIABASE_API CAerialBlock : CAerialBaseObject
{
public:
	static CAerialStrip*    CreateStripObject();
	static void             FreeStripObject(CAerialStrip* lpStrip);
	static CAerialPhoto*    CreatePhotoObject();
	static void             FreePhotoObject(CAerialPhoto* lpPhoto);

	static bool             SpaceIntersection(CAerialPhoto** lppPhts, float* pix_x, float* pix_y, int nPts, double* X, double* Y, double* Z);


	CAerialBlock();
	virtual ~CAerialBlock();

	void                      Reset();

	//工程描述
	void                      SetName(const char* lpszName);
	const char*               GetName()const;

	//工程作业人员
	void                      SetOperator(const char* lpszOperator);
	const char*               GetOperator()const;

	//工程日志文件
	void                      SetLogFile(const char* lpszLogFile);
	const char*               GetLogFile( );

	void                      SetObjectUnit(int type);
	int                       GetObjectUnit()const;

	void                      SetImageUnit(int  type);
	int                       GetImageUnit() const;

	void                      SetAngularUnit(int type);
	int                       GetAngularUnit() const;

	bool                      LoadFromFile(const char* lpszXML, IAerialProgress* lpProgress = NULL);
	bool                      SaveToFile(const char* lpszXML, IAerialProgress* lpProgress = NULL);

	int                       GetNumCamera() const;
	bool                      AddCamera(CCamera* lpCamera);
	bool                      DeleteCamera(CCamera* lpCamera);
	bool                      DeleteCamera(int index);	
	CCamera*                  GetCamera(int index);
	CCamera*                  GetCamera(const char* id);
    int                       GetCameraIndex(CCamera* lpCamera);
    int                       GetCameraIndex(const char* id);
	/*  相机是否被工程中的某些影像所引用  */
	bool                      IsUsingCamera(const char* id);

	int                       GetNumGPSIMU();
	int                       GetGPSIMUIndex(CGPSIMUDatum* lpPOS);
	CGPSIMUDatum*             GetGPSIMU(int index);
	CGPSIMUDatum*             GetGPSIMU(const char* id);
	void                      AddGPSIMU(CGPSIMUDatum* lpPOS);
	void                      DeleteGPSIMU(CGPSIMUDatum* lpPOS);


    int                       GetNumGCP();
    int                       GetGCPIndex(CGCPDatum* lpGCP);
    CGCPDatum*                GetGCP(int index);
    CGCPDatum*                GetGCP(const char* id);
    void                      AddGCP(CGCPDatum* lpGCP);
    void                      DeleteGCP(CGCPDatum* lpGCP);

	int                       GetNumPhoto();
	int                       GetPhotoIndex(CAerialPhoto* lpPhoto);
	CAerialPhoto*             GetPhoto(int index);
	CAerialPhoto*             GetPhoto(const char* id);
	void                      AddPhoto(CAerialPhoto* lpPhoto);
	void                      DeletePhoto(CAerialPhoto* lpPhoto);
	bool                      IsUsingPhoto(const char* lpszID);
	
	int                       GetNumStrip() const;
	bool                      AddStrip(CAerialStrip* lpStrip);
	int                       GetStripIndex(CAerialStrip* lpStrip);
	bool                      DeleteStrip(CAerialStrip* lpStrip);
	bool                      DeleteStrip(int index);
	CAerialStrip*             GetStrip( int index );
	CAerialStrip*             GetStrip(const char* lpszID);


	//设置像点量测精度，自动和手工量测两种
	void                      SetImagePointStdDev(float manual, float automatic);
	void                      GetImagePointStdDev(float&manual, float&automatic);

	//设置控制点精度，分为5组，0表示标准组，1组-4组，精度分为平面和高程
	void                      SetObjectPointClassStdDev(int cls, float plan, float Height);
	void                      GetObjectPointClassStdDev(int cls, float&plan, float&Height);

	//设置GPS的精度
    void                      SetGPSStdDev(float X, float Y, float Z);
	void                      GetGPSStdDev(float&X, float&Y, float&Z);

	//设置IMU的精度
	void                      SetIMUStdDev(float Omega, float Phi, float Kappa);
	void                      GetIMUStdDev(float&Omega, float&Phi, float&Kappa);


	void                      SetRefrectCorrect(bool bCorrect) { m_bRefrectCorrect = bCorrect; }
	bool                      IsRefrectCorrect() const { return m_bRefrectCorrect; }

	void                      SetCurveCorrect(bool bCorrect) { m_bCurveCorrect = bCorrect; }
	bool                      IsCurveCorrect() const { return m_bCurveCorrect; }


	//对象利用ID进行排序，以便快速获取对象指针,目前还是为空
	void                      SortPhotos();
	void                      SortGPSIMUs();
	void                      SortGCPs();
	void                      SortStrips();

	//implementation of CAeriaBaseObject
	virtual bool SaveXML(void* lpNode);
	virtual bool FromXML(void* lpNode);

protected:
	bool         IsCameraExist(CCamera* lpCam);
    bool         IsStripExist(CAerialStrip* lpStrip);

private:
	static int  ComparePhotos(const void * p1, const void * p2);
	static int  CompareGPSIMUs(const void * p1, const void * p2);
	static int  CompareGCPs(const void * p1, const void * p2);
	static int  CompareStrips(const void * p1, const void * p2);


private:
	char                                  m_szBlkName[64];
	char                                  m_szOperator[64];
	char                                  m_szLogFile[512];

    //记录工程中的所有控制点数据
    std::vector<CGCPDatum*>               m_GCPList;   
	//记录工程中的所有POS数据
	std::vector<CGPSIMUDatum*>            m_GPSIMUList;
	//记录工程中的所有影像对象
	std::vector<CAerialPhoto*>            m_PhotoList;
	//记录工程中的所有航带对象
	std::vector<CAerialStrip*>            m_StripList;

	//大气折射改正
	bool                                  m_bRefrectCorrect;
	//地球曲率改正
	bool                                  m_bCurveCorrect;

	int                                   m_nObjectUnit;  /* 0-米*/
	int                                   m_nImageUnit;   /* 0-毫米  1-微米*/
	int                                   m_nAngularUnit; /* 0-度 1-弧度 2-400进制弧度*/

	float                                 m_ImagePointStdDev[2];
	float                                 m_ObjectPointStdDev[5][2];
	float                                 m_GPSStdDev[3];
	float                                 m_IMUStdDev[3];


	//记录相机定义文件
	CCameraXMLFile                        m_CamerasFile; 
};


//////////////////////////////////////////////////////////////////////////
//   连接点相关类
//////////////////////////////////////////////////////////////////////////
class CTPCMemFile;
class CTPCPoint;
class CTPCPhotoObservations;

//每个观测类
class AERIABASE_API CTPCObservation
{
	friend class CTPCMemFile;
	friend class CTPCPoint;
	friend class CTPCPhotoObservations;
public:
	static CTPCObservation* CreateTPCObservation();
	static CTPCObservation* CreateTPCObservation(CTPCPoint* lpTPC);
	static void FreeTPCObservation(CTPCObservation* lpTPCObs);

	CTPCPoint* GetTPCPoint();
	void       SetTPCPoint(CTPCPoint* lpTPC);

	const char* GetID();

	void   SetClass(int cls);
	int    GetClass();

	void  SetPixelCoordinates(float xi, float yi);
	void  GetPixelCoordinates(float& xi, float& yi);

	//void  SetImageCoordinates(float xp, float yp);
	//void  GetImageCoordinates(float&xp, float&yp);

	void  SetImageResiduals(float resx, float resy);
	void  GetImageResiduals(float& resx, float& resy);

	CTPCObservation* operator=(const CTPCObservation* lpObs);

protected:
	CTPCObservation();
	CTPCObservation(CTPCPoint* lpTPC);
	CTPCObservation(const CTPCObservation* lpObs);
	CTPCObservation(float xi, float yi/*, float xp, float yp*/);

private:
	CTPCPoint* m_lpTPC;
    int   m_nClass;
	float m_PixelCoordinates[2];
	//float m_ImageCoordinates[2];
	float m_ImageResiduals[2];
};

//每一个连接点类
class AERIABASE_API CTPCPoint
{
	friend class CTPCMemFile;

	typedef struct tagTPCObs
	{
		CAerialPhoto* lpPht;
		CTPCObservation* lpObs;
	}TPCOBS;

public:
	static CTPCPoint* CreateTPCPoint();
	static CTPCPoint* CreateTPCPoint(const char* lpszID);
	static void       FreeTPCPoint(CTPCPoint* lpTPC);

	const char*   GetID();
	void          SetID(const char* lpszID);	

	bool          AddObservation(CAerialPhoto* lpPht, CTPCObservation* lpObs,bool bSort = false);
	bool          DeleteObservation(CAerialPhoto* lpPht);
	int           GetNumObservations();
	//获取第index个观测，返回是影像对象和观测值对象
	bool          GetObservation(int index, CAerialPhoto** lppPht, CTPCObservation** lppObs);
	//从指定的影像上获取观测值
	bool          GetObservation(CAerialPhoto* lpPht, CTPCObservation** lppObs);
	bool          GetObservation(const char* lpszID, CTPCObservation** lppObs);

	int           GetObservationIndex(CAerialPhoto* lpPht);
	bool          DeleteObservation(int index);

	//对TPCOBS按照影像ID顺序进行排序
	void          SortPhotos();

	void          GetXYZ(double&X, double&Y, double&Z);
	void          SetXYZ(double X, double Y, double Z);

	void          SetRGB(unsigned char R, unsigned char G, unsigned char B);
	void          GetRGB(unsigned char& R, unsigned char& G, unsigned char & B);
	unsigned char* GetRGB() { return (unsigned char*)m_Color; }

protected:
	CTPCPoint();
	CTPCPoint(const char* lpszID);

private:
	static int TPCOBSCompare(const void * p1, const void * p2);

protected:
	char                  m_szID[64];
	std::vector<TPCOBS>   m_PhotoObs;
	double                m_XYZ[3];
	unsigned  char        m_Color[3];
};

class AERIABASE_API CTPCPhotoObservations
{
	friend class CTPCMemFile;
public:

	CAerialPhoto*  GetPhoto();

	int              GetNumObservations();
	CTPCObservation* GetObservation(const int index);
	CTPCObservation* GetObservation(const char* lpszID);

	//对TCPObserve按照ID升序排列
	void    SortObservationID();
	void    SortObservationIDNumber();

protected:
	CTPCPhotoObservations(CAerialPhoto* lpPhoto);
	~CTPCPhotoObservations();

public:
	bool AddObservation(CTPCObservation* lpTPCObs, bool bSort = false);
	bool DeleteObservation(const char* lpszID);
	bool DeleteObservation(CTPCObservation* lpObs);	
	int  GetObservationIndex(const char* lpszID);

private:
	static int  TPCObservationCompare(const void * p1, const void * p2);
	static int  TPCObservationNumberCompare(const void * p1, const void * p2);

protected:
	CAerialPhoto* m_lpAerialPhoto;
	std::vector<CTPCObservation*> m_TPCObservationList;
};

//连接点保存文件类
class AERIABASE_API CTPCMemFile
{
public:
	CTPCMemFile();
	~CTPCMemFile();

	bool   Create(CAerialBlock* lpAerialBlock);
	bool   LoadFromFile(CAerialBlock* lpAerialBlock, const char* lpszFileName, IAerialProgress* lpProgress = NULL);
	bool   SaveToFile(const char* lpszFileName, IAerialProgress* lpProgress = NULL);
	void   Close();

    CAerialBlock*  GetAerialBlock();

	void           Clear();

	int            GetNumTPCPoint();
	CTPCPoint*     GetTPCPoint(int index);
	CTPCPoint*     GetTPCPoint(const char* lpszID);

    int            GetNumObservations();

	

	bool           AddTPCPoint(CTPCPoint* lpPoint, bool bAddObs = true, bool bSortTPC = false);
	bool           DeleteTPCPoint(CTPCPoint* lpPoint);
	bool           DeleteTPCPoint(const char* lpszID);

	CTPCPhotoObservations* GetPhotoObservations(const char* lpszPhtID);
	CTPCPhotoObservations* GetPhotoObservations(CAerialPhoto* lpPht);

	int                    GetNumPhotoObservations();
	CTPCPhotoObservations* GetPhotoObservations(int index);

	void           SortTPCPoints(); //对点进行排序
	void           SortPhotos();    //对影像ID进行升序排序


	bool           ImportPixelGridTPC(const char* lpszDir);

	//void           ThinPhotoOverlapNum(int maxOverlap = 260);

	int            GetTPCIndex(CTPCPoint* lpPoint); //
	int            GetTPCIndex(const char* lpszID); //
protected:

	bool           BuildTPCFromObservations();

private:
	static int  TPCPointCompare(const void * p1, const void * p2);
	static int  TPCPhotoObservationsCompare(const void * p1, const void * p2);
	static int  TPCNodeCompare(const void * p1, const void * p2);

protected:
	std::vector<CTPCPhotoObservations*>    m_PhotoObsList;
	std::vector<CTPCPoint*>                m_TPCPointList;

private:
    CAerialBlock* m_lpAerialBlock;
};



#define M_PI (3.1415926535897932384626433832795)
#define D2A (0.01745329251994329576923690768489)
#define A2D (57.295779513082320876798154814105)

AERIABASE_API void XRotMatrix(double* rotmatrix, double omega, double phi, double kappa);
AERIABASE_API void YRotmatrix(double* rotmatrix, double phi, double omega, double kappa);

AERIABASE_API void GetXRotAngle(double* rotmatrix, double &omega, double &phi, double &kappa);
AERIABASE_API void GetYRotAngle(double* rotmatrix, double &phi,   double &omega, double &kappa);




#endif //__AERIABASE_H__