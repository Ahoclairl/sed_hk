/***************************************************************************************************
*
* ��Ȩ��Ϣ����Ȩ���� (c) 2022, ���ݺ��������˼������޹�˾, ��������Ȩ��
*
* �ļ����ƣ�Mv3dRgbdApilEx.h
* ժ    Ҫ: RGBD ���SDK����չ�ӿڶ���
*
* ��ǰ�汾��1.1.0
* ��    ��: �����Ӿ�SDK�Ŷ�
* ��    �ڣ�2022.4.26
* ��    ע���½�
****************************************************************************************************/

#ifndef _MV3D_RGBD_API_EX_H_
#define _MV3D_RGBD_API_EX_H_

#include "Mv3dRgbdApi.h"
#include "Mv3dRgbdDefine.h"


#ifdef __cplusplus
extern "C" {
#endif 

/************************************************************************
*  @fn         MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_CamIsAccessible(IN MV3D_RGBD_DEVICE_INFO* pstDevInfo, IN unsigned int nAccessMode)
*  @brief      �ж�����ķ���Ȩ��
*  @param      MV_STEREOCAM_NET_INFO* pstDevInfo                [IN]        �����Ϣ
*  @param      unsigned int * nAccessMode                       [IN]        ����ķ���Ȩ��      //Eg: MV3D_RGBD_ACCESS_Exclusive
*  @return    �ɴ����true�����ɴ����false 

*  @fn         MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_CamIsAccessible(IN MV3D_RGBD_DEVICE_INFO* pstDevInfo, IN unsigned int nAccessMode)
*  @brief      Get camera's Access privileges
*  @param      MV3D_RGBD_DEVICE_INFO* pstDevInfo                [IN]        camera info 
*  @param       unsigned int * nAccessMode                      [IN]        camera's Access privileges    //Eg: MV3D_RGBD_ACCESS_Exclusive
*  @return     Success, return true. Failure, false
************************************************************************/    
MV3D_RGBD_API MV3D_RGBD_STATUS  MV3D_RGBD_CamIsAccessible(MV3D_RGBD_DEVICE_INFO* pstDevInfo,unsigned int nAccessMode);

/************************************************************************
*  @fn     MV3D_RGBD_GetCameraXML
*  @brief  ��ȡ���xml
*  @param  handle                      [IN]            �豸���
*  @param  pData                       [IN OUT]        ����xml����
*  @param  nDataSize                   [IN]............���泤��
*  @param  pnDataLen                   [OUT]...........����ʵ�ʳ���
*  @return �ɹ�,����MV3D_RGBD_OK,ʧ��,���ش�����

*  @fn     MV3D_RGBD_GetCameraXML
*  @brief  Get Camera XML
*  @param  handle                      [IN]            Device handle
*  @param  pData                       [IN OUT]        recieve xml buf point
*  @param  nDataSize                   [IN]............buffer size
*  @param  pnDataLen                   [OUT]...........get data len
*  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_GetCameraXML(void * handle,unsigned char* pData,unsigned int nDataSize,unsigned int* pnDataLen);

/***********************************************************************
 *  @fn     MV3D_RGBD_XML_GetNodeAccessMode
 *  @brief  ��õ�ǰ�ڵ�ķ���ģʽ
 *  @param  handle                      [IN]            �豸���
 *  @param  pstrName                    [IN]            �ڵ�����
 *  @param  pAccessMode                 [OUT]           �ڵ�ķ���ģʽ
 *  @return �ɹ�������MV3D_RGBD_OK�����󣬷��ش�����

 *  @fn     MV3D_RGBD_XML_GetNodeAccessMode
 *  @brief  Get Access mode of cur node
 *  @param  handle                      [IN]            Device handle
 *  @param  pstrName                    [IN]            Name of node
 *  @param  pAccessMode                 [OUT]           Access mode of the node
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ***********************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_XML_GetNodeAccessMode(void* handle,const char * pstrName,MV3D_RGBD_XML_AccessMode *pAccessMode);

/***********************************************************************
 *  @fn     MV3D_RGBD_SetImageNodeNum
 *  @brief  ����SDK�ڲ�ͼ�񻺴�ڵ���������ڵ���1����ץͼǰ����
 *  @param  handle                      [IN]            �豸���
 *  @param  nNum                        [IN]            ����ڵ����
 *  @return �ɹ�������MV3D_RGBD_OK�����󣬷��ش�����
 
 *  @fn     MV3D_RGBD_SetImageNodeNum
 *  @brief  Set the number of the internal image cache nodes in SDK, Greater than or equal to 1, to be called before the capture
 *  @param  handle                      [IN]            Device handle
 *  @param  nNum                        [IN]            Image Node Number
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ***********************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_SetImageNodeNum(void* handle,unsigned int nNodeCount);

/************************************************************************
 *  @fn     MV3D_RGBD_InvalidateNodes
 *  @brief  ���GenICam�ڵ㻺��
 *  @param  handle                      [IN]            �豸���
 *  @return �ɹ�������MV3D_RGBD_OK�����󣬷��ش�����
 
 *  @fn     MV3D_RGBD_InvalidateNodes
 *  @brief  Invalidate GenICam Nodes
 *  @param  handle                      [IN]            Device handle
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_InvalidateNodes(void* handle);

/************************************************************************
*  @fn     MV3D_RGBD_ReadAddressMem
*  @brief  ���ڴ�
*  @param  handle                      [IN]            �豸���
*  @param  pBuffer                     [IN][OUT]       ��Ϊ����ֵʹ�ã�����������ڴ�ֵ���ڴ�ֵ�ǰ��մ��ģʽ�洢�ģ�
*  @param  nRegAddress                 [IN]            ����ȡ���ڴ��ַ���õ�ַ���Դ��豸��Camera.xml�ļ��л�ȡ������xxx_RegAddr��xml�ڵ�ֵ
                                                       ���豸��Camera.xml�ļ������豸��֮���Զ�������Ӧ�ó���ĵ�ǰĿ¼�У�
*  @param  nLength                     [IN]            ����ȡ���ڴ泤��
*  @return �ɹ�,����MV3D_RGBD_OK,ʧ��,���ش�����

*  @fn     MV3D_RGBD_ReadAddressMem
*  @brief  Read Memory
*  @param  handle                      [IN]            Device Handle
*  @param  pBuffer                     [IN][OUT]       Used as a return value, save the read-in memory value ( Memory value is stored in accordance with the big end model)
*  @param  nRegAddress                 [IN]            Memory address to be read, which can be obtained from the Camera.xml file of the device, the form xml node value of xxx_RegAddr
                                                       (Camera.xml file of device is automatically generated in the application's current directory after the device is opened)
*  @param  nLength                     [IN]            Length of the memory to be read
*  @return Success, return MV3D_RGBD_OK. Failure, return error code 
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_ReadAddressMem(void * handle,void *pBuffer,int64_t nRegAddress,int64_t nLength);

/************************************************************************
*  @fn     MV3D_RGBD_WriteAddressMem
*  @brief  д�ڴ�
*  @param  handle                      [IN]            �豸���
*  @param  pBuffer                     [IN]            ��д����ڴ�ֵ��ע���ڴ�ֵҪ���մ��ģʽ�洢��
*  @param  nRegAddress                 [IN]            ��д����ڴ��ַ���õ�ַ���Դ��豸��Camera.xml�ļ��л�ȡ������xxx_RegAddr��xml�ڵ�ֵ
                                                       ���豸��Camera.xml�ļ������豸��֮���Զ�������Ӧ�ó���ĵ�ǰĿ¼�У�
*  @param  nLength                     [IN]            ��д����ڴ泤��
*  @return �ɹ�,����MV3D_RGBD_OK,ʧ��,���ش�����

*  @fn     MV3D_RGBD_WriteAddressMem
*  @brief  Write Memory
*  @param  handle                      [IN]            Device Handle
*  @param  pBuffer                     [IN]            Memory value to be written ( Note the memory value to be stored in accordance with the big end model)
*  @param  nRegAddress                 [IN]            Memory address to be written, which can be obtained from the Camera.xml file of the device, the form xml node value of xxx_RegAddr
                                                       (Camera.xml file of device is automatically generated in the application's current directory after the device is opened)
*  @param  nLength                     [IN]            Length of the memory to be written
*  @return Success, return MV3D_RGBD_OK. Failure, return error code 
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_WriteAddressMem(void * handle,void *pBuffer,int64_t nRegAddress,int64_t nLength);

/************************************************************************
*  @fn     MV3D_RGBD_FileAccessRead()
*  @brief  �������ȡ�ļ�
*  @param  handle                [IN]           �����ַ
*  @param  pstFileAccess         [IN]           �ļ���ȡ�ṹ��
*  @return �ɹ�������MV3D_RGBD_OK�����󣬷��ش����� 

*  @fn     MV3D_RGBD_FileAccessRead()
*  @brief  Read the file from the camera
*  @param  handle                [IN]           Handle
*  @param  pstFileAccess         [IN]           File access structure
*  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_FileAccessRead(void* handle,MV3D_RGBD_FILE_ACCESS * pstFileAccess);

/************************************************************************
*  @fn     MV3D_RGBD_FileAccessWrite()
*  @brief  ���ļ�д�����
*  @param  handle                [IN]           �����ַ
*  @param  pstFileAccess         [IN]           �ļ���ȡ�ṹ��
*  @return �ɹ�������MV3D_RGBD_OK�����󣬷��ش����� 

*  @fn     MV3D_RGBD_FileAccessWrite()
*  @brief  Write the file to camera
*  @param  handle                [IN]           Handle
*  @param  pstFileAccess         [IN]           File access structure
*  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_FileAccessWrite(void* handle,MV3D_RGBD_FILE_ACCESS * pstFileAccess);

/************************************************************************
 *  @fn     MV3D_RGBD_SetGvcpTimeout
 *  @brief  ����GVCP���ʱʱ��
 *  @param  handle                      [IN]            �豸���
 *  @param  nMillisec                   [IN]            ��ʱʱ�䣬�Ժ���λ��λ����Χ��0-10000
 *  @return �ɹ�������MV3D_RGBD_OK�����󣬷��ش����� 

 *  @fn     MV_GIGE_SetGvcpTimeout
 *  @brief  Set GVCP cammand timeout
 *  @param  handle                      [IN]            Device handle
 *  @param  nMillisec                   [IN]            Timeout, ms as unit, range: 0-10000
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_SetGvcpTimeout(void* handle, unsigned int nMillisec);

/************************************************************************
*  @fn     MV3D_RGBD_GetGvcpTimeout
*  @brief  ��ȡGVCP���ʱʱ��
*  @param  handle                      [IN]            �豸���
*  @param  pMillisec                   [IN]            ��ʱʱ��ָ�룬�Ժ���λ��λ
*  @return �ɹ�������MV3D_RGBD_OK�����󣬷��ش����� 

*  @fn     MV_GIGE_GetGvcpTimeout
*  @brief  Get GVCP cammand timeout
*  @param  handle                      [IN]            Device handle
*  @param  pMillisec                   [IN]            Timeout, ms as unit
*  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_GetGvcpTimeout(void* handle, unsigned int* pMillisec);




#ifdef __cplusplus
}
#endif 

#endif //_MV3D_RGBD_API_EX_H_