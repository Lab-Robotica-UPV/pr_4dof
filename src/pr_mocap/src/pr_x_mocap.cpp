#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "pr_mocap/pr_x_mocap.hpp"
#include "pr_lib/pr_model.hpp"

using std::placeholders::_1;

namespace pr_mocap
{

    PRXMocap::PRXMocap(const rclcpp::NodeOptions & options)
    : Node("pr_x_mocap", options)
    {
        this->declare_parameter<double>("tol", 0.01);
        this->declare_parameter<std::string>("server_address", "158.42.21.85");
        this->declare_parameter<int>("server_command_port", 1510);
        this->declare_parameter<int>("server_data_port", 1511);
        this->declare_parameter<std::vector<std::string>>("marker_names", {"P_Movil1_1", "P_Movil1_2", "P_Movil1_3", "P_Fija1_1", "P_Fija1_2", "P_Fija1_3"});
        this->declare_parameter<bool>("robot_5p", true);

        this->get_parameter("server_address", server_address);
        this->get_parameter("server_command_port", server_command_port);
        this->get_parameter("server_data_port", server_data_port);
        this->get_parameter("marker_names", markers_name);
        this->get_parameter("robot_5p", robot_5p);

        publisher_ = this->create_publisher<pr_msgs::msg::PRMocap>(
            "x_coord_mocap",
            1
        );

        unsigned char ver[4];
        NatNet_GetVersion(ver);
        printf( "NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3] );
        //NatNet client
        g_pClient = new NatNetClient();
        g_pClient->SetFrameReceivedCallback(DataHandler, this);

        g_connectParams.connectionType = ConnectionType_Multicast;
        g_connectParams.serverCommandPort = server_command_port;
        g_connectParams.serverDataPort = server_data_port;
        g_connectParams.serverAddress = server_address.c_str();
        g_connectParams.localAddress = "";
        g_connectParams.multicastAddress = "";

        int iResult = ConnectClient();

        if (iResult != ErrorCode_OK)
            printf("Error initializing client.  See log for details.  Exiting");
        else
            printf("Client initialized and ready.\n");

        //Retrieve Data Descriptions from Motive
        void* response;
        int nBytes;
        printf("[SampleClient] Sending Test Request\n");
        iResult = g_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
        if (iResult == ErrorCode_OK)
        {
            printf("[SampleClient] Received: %s", (char*)response);
        }

        // Retrieve Data Descriptions from Motive
        printf("\n\n[SampleClient] Requesting Data Descriptions...");
        sDataDescriptions* pDataDefs = NULL;
        iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
        if (iResult != ErrorCode_OK || pDataDefs == NULL)
        {
            printf("[SampleClient] Unable to retrieve Data Descriptions.");
        }
        else
        {
            for (int i=0; i < pDataDefs->nDataDescriptions; i++)
            {
                if(pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
                {
                    // MarkerSet
                    sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                    if (strcmp(pMS->szName, "all")==0){
                        printf("\nMarkerSet Name : %s\n", pMS->szName);
                        for(int i=0; i < pMS->nMarkers; i++){
                            printf("%s\n", pMS->szMarkerNames[i]);
                        
                            for(int j=0; j<6; j++){
                                if(strcmp(pMS->szMarkerNames[i], markers_name[j].c_str())==0)
                                    markers_pos[j] = i;
                            }
                        }
                    printf("\nIDS: %d, %d, %d, %d, %d, %d\n", markers_pos[0], markers_pos[1],markers_pos[2],markers_pos[3],markers_pos[4],markers_pos[5]);
                    }
                }
            }
        }
    }

    PRXMocap::~PRXMocap()
    {
        if (g_pClient){
		    g_pClient->Disconnect();
		    delete g_pClient;
		    g_pClient = NULL;
	    }
    }

    int PRXMocap::ConnectClient()
    {
        // Release previous server
        g_pClient->Disconnect();

        // Init Client and connect to NatNet server
        int retCode = g_pClient->Connect( g_connectParams );
        if (retCode != ErrorCode_OK)
        {
            printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
            return ErrorCode_Internal;
        }
        else
        {
            // connection succeeded

            void* pResult;
            int nBytes = 0;
            ErrorCode ret = ErrorCode_OK;

            // print server info
            memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
            ret = g_pClient->GetServerDescription( &g_serverDescription );
            if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
            {
                printf("Unable to connect to server. Host not present. Exiting.");
                return 1;
            }
            printf("\n[SampleClient] Server application info:\n");
            printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
                g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
            printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
                g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
            printf("Client IP:%s\n", g_connectParams.localAddress );
            printf("Server IP:%s\n", g_connectParams.serverAddress );
            printf("Server Name:%s\n", g_serverDescription.szHostComputerName);

            // get mocap frame rate
            ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
            if (ret == ErrorCode_OK)
            {
                float fRate = *((float*)pResult);
                printf("Mocap Framerate : %3.2f\n", fRate);
            }
            else
                printf("Error getting frame rate.\n");

            // get # of analog samples per mocap frame of data
            ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
            if (ret == ErrorCode_OK)
            {
                g_analogSamplesPerMocapFrame = *((int*)pResult);
                printf("Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
            }
            else
                printf("Error getting Analog frame rate.\n");
        }

        return ErrorCode_OK;
    }

}

    void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)
    {
        using namespace pr_mocap;
        
        PRXMocap* pr_x_mocap = (PRXMocap*) pUserData;
        NatNetClient* pClient = pr_x_mocap->g_pClient;

        // Software latency here is defined as the span of time between:
        //   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
        // and
        //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
        //
        // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
        // because it additionally includes the time spent preparing to stream the data via NatNet.
        const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
        const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(pr_x_mocap->g_serverDescription.HighResClockFrequency);

        // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
        // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
        const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->TransmitTimestamp ) * 1000.0;

        //printf("FrameID : %d\n", data->iFrame);
        //printf("Timestamp : %3.2lf\n", data->fTimestamp);
        //printf("Software latency : %.2lf milliseconds\n", softwareLatencyMillisec);

        //Update latency on the topic message
        pr_x_mocap->mocap_msg.latency = softwareLatencyMillisec;

        // Only recent versions of the Motive software in combination with ethernet camera systems support system latency measurement.
        // If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
        const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

        if ( bSystemLatencyAvailable )
        {
            // System latency here is defined as the span of time between:
            //   a) The midpoint of the camera exposure window, and therefore the average age of the photons (CameraMidExposureTimestamp)
            // and
            //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
            const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
            const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(pr_x_mocap->g_serverDescription.HighResClockFrequency);

            // Client latency is defined as the sum of system latency and the transit time taken to relay the data to the NatNet client.
            // This is the all-inclusive measurement (photons to client processing).
            const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->CameraMidExposureTimestamp ) * 1000.0;

            // You could equivalently do the following (not accounting for time elapsed since we calculated transit latency above):
            //const double clientLatencyMillisec = systemLatencyMillisec + transitLatencyMillisec;

            //printf( "System latency : %.2lf milliseconds\n", systemLatencyMillisec );
            //printf( "Total client latency : %.2lf milliseconds (transit time +%.2lf ms)\n", clientLatencyMillisec, transitLatencyMillisec );
        }
        else
        {
            //printf( "Transit latency : %.2lf milliseconds\n", transitLatencyMillisec );
        }

        // FrameOfMocapData params
        bool bIsRecording = ((data->params & 0x01)!=0);
        bool bTrackedModelsChanged = ((data->params & 0x02)!=0);
        //if(bIsRecording)
            //printf("RECORDING\n");
        //if(bTrackedModelsChanged)
            //printf("Models Changed.\n");
        

        // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
        int hour, minute, second, frame, subframe;
        NatNet_DecodeTimecode( data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe );
        // decode to friendly string
        char szTimecode[128] = "";
        NatNet_TimecodeStringify( data->Timecode, data->TimecodeSubframe, szTimecode, 128 );
        //printf("Timecode : %s\n", szTimecode);

        // labeled markers - this includes all markers (Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
        bool bOccluded;     // marker was not visible (occluded) in this frame
        bool bPCSolved;     // reported position provided by point cloud solve
        bool bModelSolved;  // reported position provided by model solve
        bool bHasModel;     // marker has an associated asset in the data stream
        bool bUnlabeled;    // marker is 'unlabeled', but has a point cloud ID that matches Motive PointCloud ID (In Motive 3D View)
        bool bActiveMarker; // marker is an actively labeled LED marker

        //printf("Markers [Count=%d]\n", data->nLabeledMarkers);

        for(int i=0; i < data->nLabeledMarkers; i++)
        {
            bOccluded = ((data->LabeledMarkers[i].params & 0x01)!=0);
            bPCSolved = ((data->LabeledMarkers[i].params & 0x02)!=0);
            bModelSolved = ((data->LabeledMarkers[i].params & 0x04) != 0);
            bHasModel = ((data->LabeledMarkers[i].params & 0x08) != 0);
            bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
            bActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);

            sMarker marker = data->LabeledMarkers[i];

            // Marker ID Scheme:
            // Active Markers:
            //   ID = ActiveID, correlates to RB ActiveLabels list
            // Passive Markers: 
            //   If Asset with Legacy Labels
            //      AssetID 	(Hi Word)
            //      MemberID	(Lo Word)
            //   Else
            //      PointCloud ID
            int modelID, markerID;
            NatNet_DecodeID( marker.ID, &modelID, &markerID );
            
            char szMarkerType[512];
            if (bActiveMarker)
                strcpy(szMarkerType, "Aceetive");
            else if(bUnlabeled)
                strcpy(szMarkerType, "Unlabeled");
            else
                strcpy(szMarkerType, "Labeled");

            //printf("%s Marker [ModelID=%d, MarkerID=%d, Occluded=%d, PCSolved=%d, ModelSolved=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
            //    szMarkerType, modelID, markerID, bOccluded, bPCSolved, bModelSolved,  marker.size, marker.x, marker.y, marker.z);
            
            //Fill Markers Matrix
            for (int j=0; j<6; j++){
                if (i == pr_x_mocap->markers_pos[j]){
                    pr_x_mocap->MarkersMatrix(0,j) = marker.x;
                    pr_x_mocap->MarkersMatrix(1,j) = marker.y;
                    pr_x_mocap->MarkersMatrix(2,j) = marker.z;
                }
            }
            // if (pr_x_mocap->iter%200 == 0){
            //     std::cout << "Marker 1: ";
            //     for (int k=0; k<3; k++){
            //         std::cout << pr_x_mocap->MarkersMatrix(k,0) << " ";
            //     }
            //     std::cout << std::endl;
            //     std::cout << "Marker 2: ";
            //     for (int k=0; k<3; k++){
            //         std::cout << pr_x_mocap->MarkersMatrix(k,1) << " ";
            //     }
            //     std::cout <<std::endl;
            //     std::cout << "Marker 3: ";
            //     for (int k=0; k<3; k++){
            //         std::cout << pr_x_mocap->MarkersMatrix(k,2) << " ";
            //     }
            //     std::cout << std::endl;
            //     std::cout << "Marker 4: ";
            //     for (int k=0; k<3; k++){
            //         std::cout << pr_x_mocap->MarkersMatrix(k,3) << " ";
            //     }
            //     std::cout << std::endl;
            //     std::cout << "Marker 5: ";
            //     for (int k=0; k<3; k++){
            //         std::cout << pr_x_mocap->MarkersMatrix(k,4) << " ";
            //     }
            //     std::cout << std::endl;
            //     std::cout << "Marker 6: ";
            //     for (int k=0; k<3; k++){
            //         std::cout << pr_x_mocap->MarkersMatrix(k,5) << " ";
            //     }
            //     std::cout << std::endl;
            //     //std::cout << sqrt(pow(pr_x_mocap->MarkersMatrix(0,0)-pr_x_mocap->MarkersMatrix(0,1),2) + pow(pr_x_mocap->MarkersMatrix(1,0)-pr_x_mocap->MarkersMatrix(1,1),2) + pow(pr_x_mocap->MarkersMatrix(2,0)-pr_x_mocap->MarkersMatrix(2,1),2)) << std::endl;
            // }
            pr_x_mocap->iter++;
        }

        //Get coordinates from the markers
        PRModel::OptiTrack::PosOriPM(pr_x_mocap->XCoords,
                                     pr_x_mocap->MarkersMatrix.col(3),
                                     pr_x_mocap->MarkersMatrix.col(4),
                                     pr_x_mocap->MarkersMatrix.col(5),
                                     pr_x_mocap->MarkersMatrix.col(0),
                                     pr_x_mocap->MarkersMatrix.col(1),
                                     pr_x_mocap->MarkersMatrix.col(2),
                                     pr_x_mocap->robot_5p);

        pr_x_mocap->mocap_msg.x_coord.data[0] = pr_x_mocap->XCoords(0, 0);
        pr_x_mocap->mocap_msg.x_coord.data[1] = pr_x_mocap->XCoords(2, 0);
        pr_x_mocap->mocap_msg.x_coord.data[2] = pr_x_mocap->XCoords(1, 1);
        pr_x_mocap->mocap_msg.x_coord.data[3] = pr_x_mocap->XCoords(2, 1);

        pr_x_mocap->mocap_msg.current_time = pr_x_mocap->get_clock()->now();

        pr_x_mocap->publisher_->publish(pr_x_mocap->mocap_msg);
    }

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_mocap::PRXMocap)
