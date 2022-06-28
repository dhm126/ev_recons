#include <esvo_core/EventProvider.h>

using namespace esvo_core;
using namespace H5;
using namespace std;


eventProvider::eventProvider(ros::NodeHandle &nh,ros::NodeHandle &pnh):
    nh_(nh),
    nhp_(pnh)
{
    events_left_pub_ = nh_.advertise<dvs_msgs::EventArray>("/ep/events", 1);
    sensor_height_=tools::param(nhp_,"sensor_height",720);
    sensor_width_=tools::param(nhp_,"sensor_width",1280);
    std::string ff="/media/zhouyum/NN/loop-floor0-events_left.hdf";
    h5f_name_=tools::param(nhp_,"hdf5FileName",ff);
    
    // nhp_.param<size_t>("sensor_height",sensor_height_,720);
    // nhp_.param<size_t>("sensor_width",sensor_width_,1280);
    // nhp_.param<std::string>("hdf5FileName",h5f_name_,""); 
    
    //sensor_height_=720;
    //sensor_width_=1280;
    //h5f_name_="/media/zhouyum/NN/loop-floor0-events_left.hdf";

    std::thread EventEmiting(&eventProvider::EventReadingloop, this);                        
    EventEmiting.detach();
}

eventProvider::~eventProvider(){
    events_left_pub_.shutdown();
    events_right_pub_.shutdown();

}
void eventProvider::readH5Datasets(string fname, string dataset,vector<double > &data,hsize_t hslab_offset, hsize_t hslab_count)
{

    H5File file( fname.c_str(), H5F_ACC_RDONLY );
    //cout<<"Reading h5file "<<fname<<endl;
    
    if(!H5Lexists(file.getId(), dataset.c_str(), H5P_DEFAULT)){
        cout<<"Dataset does not exist.. skipping!"<<endl;
        return;
    }
    DataSet dset= file.openDataSet(dataset);
    //DataSet dset= file.openDataSet(dataset.c_str());
    
    //Get dataspace of the dataset.
    DataSpace dataspace = dset.getSpace();//memspace 
    // Get the number of dimensions in the dataspace.
    int rank = dataspace.getSimpleExtentNdims();//extent dims =len(dataset)

    if (rank == 0){ // for single value datasets
        // create a vector the same size as the dataset
        
        data.resize(1);
        cout<<"-------"<<"READING HDF5 FILE"<<endl;
        cout<<"Vectsize: "<<data.size()<<endl;
        cout<<"-------"<<"READING HDF5 FILE"<<endl;
        dset.read(data.data(), PredType::NATIVE_DOUBLE, H5S_ALL, H5S_ALL);
    }
    else { // for array datasets
        // Get the dimension size of each dimension in the dataspace and display them.
        hsize_t dims_out[1];
        int ndims = dataspace.getSimpleExtentDims( dims_out, NULL);
        cout<<"-------"<<"READING HDF5 FILE"<<endl;
        cout << "rank " << rank << ", dimensions " <<
                (unsigned long)(dims_out[0]) << endl;
        cout<<"ndims :"<<ndims<<endl;
        cout<<"-------"<<"READING HDF5 FILE"<<endl;
        hsize_t offset[1];
        hsize_t count[1];
        offset[0] = hslab_offset;
        count[0] = std::min(hslab_count, dims_out[0] - hslab_offset);
        // cout<<"count: "<< count[0]<<" "<<offset[0]<<endl;
        dataspace.selectHyperslab(H5S_SELECT_SET, count, offset);

        // Define the memory dataspace
        DataSpace memspace (1, count);
        
        // create a vector the same size as the dataset
        data.resize(count[0]);
        
        
        // pass pointer to the array (or vector) to read function, along with the data type and space.
        dset.read(data.data(), PredType::NATIVE_DOUBLE, memspace, dataspace);
        
        // for(auto i=data.begin();i!=data.end();i++){
        //     cout<<dataset.c_str()<<"location ="<<*i<<endl;
        // }
    }
    
    // close the HDF5 file
    file.close();
}
void eventProvider::TimeSlicer(string fname, string dataset,vector<double > &data,hsize_t &hslab_offset)
{
   
    H5File file( fname.c_str(), H5F_ACC_RDONLY );
   
    
    if(!H5Lexists(file.getId(), dataset.c_str(), H5P_DEFAULT)){
        cout<<"Dataset does not exist.. skipping!"<<endl;
        return;
    }
   
    DataSet dset= file.openDataSet(dataset.c_str());
     
    //Get dataspace of the dataset.
    DataSpace dataspace = dset.getSpace();//memspace 
    // Get the number of dimensions in the dataspace.
    int rank = dataspace.getSimpleExtentNdims();//extent dims =len(dataset)
    assert (rank!=0);
    hsize_t data_dims[1];
    int ndims = dataspace.getSimpleExtentDims(data_dims, NULL);//total extent dims 
    cout<<"total dims "<<data_dims[0]<<endl;
    //assert(hslab_offset<=ndims);
    hsize_t offset[1];
    hsize_t count[1];
    

    offset[0]=hslab_offset;
    count[0]=1;
    dataspace.selectHyperslab(H5S_SELECT_SET, count, offset);
    DataSpace space (1,count);

    data.resize(count[0]);

    dset.read(data.data(),PredType::NATIVE_DOUBLE,space,dataspace);
    hslab_offset++;
    file.close();
}
void eventProvider::readH5Datasets(string fname, string dataset,vector<double > &data,
                                   hsize_t hslab_offset, hsize_t &hslab_count,double time_limit)
{

    H5File file( fname.c_str(), H5F_ACC_RDONLY );
    //cout<<"Reading h5file "<<fname<<endl;
    
    if(!H5Lexists(file.getId(), dataset.c_str(), H5P_DEFAULT)){
        cout<<"Dataset does not exist.. skipping!"<<endl;
        return;
    }
    DataSet dset= file.openDataSet(dataset);
    //DataSet dset= file.openDataSet(dataset.c_str());
    
    //Get dataspace of the dataset.
    DataSpace dataspace = dset.getSpace();//memspace 
    // Get the number of dimensions in the dataspace.
    int rank = dataspace.getSimpleExtentNdims();//extent dims =len(dataset)

    if (rank == 0){ // for single value datasets
        // create a vector the same size as the dataset
        
        data.resize(1);
        cout<<"-------"<<"READING HDF5 FILE"<<endl;
        cout<<"Vectsize: "<<data.size()<<endl;
        cout<<"-------"<<"READING HDF5 FILE"<<endl;
        dset.read(data.data(), PredType::NATIVE_DOUBLE, H5S_ALL, H5S_ALL);
    }
    else { // for array datasets
        // Get the dimension size of each dimension in the dataspace and display them.
        hsize_t dims_out[1];
        int ndims = dataspace.getSimpleExtentDims( dims_out, NULL);
        cout<<"-------"<<"READING HDF5 FILE"<<endl;
        cout << "rank " << rank << ", dimensions " <<
                (unsigned long)(dims_out[0]) << endl;
        cout<<"ndims :"<<ndims<<endl;
        cout<<"-------"<<"READING HDF5 FILE"<<endl;
        hsize_t offset[1];
        hsize_t count[1];
        offset[0] = hslab_offset;
        count[0] = 1;//std::min(hslab_count, dims_out[0] - hslab_offset);
        vector<double> gz(1,0.);
        while( offset[0]>=hslab_offset){ //==while(true)
        dataspace.selectHyperslab(H5S_SELECT_SET,count,offset);//from offset select count region
        DataSpace memspace (1, count);
        //data.resize(1);
        
        int data_offset=offset[0]-hslab_offset;
        //TODO
        //double * buff;
        dset.read(gz.data(), PredType::NATIVE_DOUBLE, memspace, dataspace);

        cout<<"每次所读取一个时间戳 后面值应为1 "<<gz.size()<<"\n";
        cout<<"该时间戳所对应的值为"<<*gz.data()<<"\n";
        offset[0]++;
        cout<<"current time offset "<<offset[0]<<endl;
        cout<<"缓冲区的值为"<<*gz.data()<<endl;
        //data.emplace_back(gz.begin(),gz.end());
        data.insert(data.end(),gz.begin(),gz.end());
        
        if(gz.back()>=time_limit) break;
            }
        cout<<"event ts in this stream=="<<data.size()<<endl;
        hslab_count+=offset[0];
        //hslab_offset=offset[0];
        }
    
    // close the HDF5 file
    file.close();
}
void eventProvider::EventReadingloop(){
   //never stop
   //packet size== every vector<events > .size() in dvs_msgs::EventArray
     int numev_per_bag = 100000; //500M
     int packet_size = 10000;
     hsize_t numev_per_stream=0;
     
     numev_per_stream=numev_per_bag;
    /*
        * Try block to detect exceptions raised by any of the calls inside it
        */
    ros::Rate r(30);
    try
    {
        hsize_t hslab_offset = 0;
        hsize_t t_cnt=0;

        int frame_id=0;
        while(ros::ok()){
            ros::Time t_begin=ros::Time::now();
            std::vector<double> data2;
            //TimeSlicer(h5f_name_,"ms_to_idx",data2,t_cnt);
            //if(data2.size()==1)
            //cout<<"这个idx对应的时间戳 time_to_idx为"<<data2[0]<<endl;
            //t_cnt++;
            //cout<<"time count 来到了第  "<<t_cnt<<"  个时间点"<<endl;

            std::vector<double> data;
            readH5Datasets(h5f_name_, "events/t", data, hslab_offset, numev_per_stream);
            //assert(numev_per_stream!=0);
            std::vector<double> t(data);
            //cout<<"angel"<<t.size()<<endl;
            data.resize(0);
            readH5Datasets(h5f_name_, "events/x", data, hslab_offset, numev_per_stream);
            std::vector<uint16_t> x(data.begin(), data.end());
            
            data.resize(0);
            readH5Datasets(h5f_name_, "events/y", data, hslab_offset, numev_per_stream);
            
            std::vector<uint16_t> y(data.begin(), data.end());
            data.resize(0);
            readH5Datasets(h5f_name_, "events/p", data, hslab_offset, numev_per_stream);
            
            std::vector<uint8_t> p(data.begin(), data.end());
            data.resize(0);
            
            
            ros::Time t_end=ros::Time::now();
            cout<<(t_end-t_begin).toSec()<<"\n";

            //float t_offset;
            // if (offset_data.size()>0)
            //     t_offset = offset_data[0];
            // else
            //     t_offset = 0;

            //cout<<"Writing to rosbag #"<<i<<" at offset "<<hslab_offset<<" containing "<<t.size()<<" events"<<endl;
            // Write to rosbag
           // rosbag::Bag bag;
            //bag.open(bagname+"_"+std::to_string(i)+".bag", rosbag::bagmode::Write);
            dvs_msgs::EventArray evQueue;
            dvs_msgs::EventArrayPtr evPtr(new dvs_msgs::EventArray());

            for (int i=0; i<t.size(); i++){
                dvs_msgs::Event ev;
                ev.ts.fromSec(t[i]/1e6);
                // ev.ts.fromSec((t[i]+t_offset)/1e6);
                ev.x = x[i];
                ev.y = y[i];
                ev.polarity = (int)p[i];
                evPtr->events.push_back(ev); 
                }
                
//                    cout<<"writing msg "<<i*100./t.size()<<"%"<<endl;
                    evPtr->header.stamp=ros::Time::now();
                    evPtr->header.frame_id=frame_id;
                    evPtr->height=sensor_height_;
                    evPtr->width=sensor_width_;

                    // evQueue.header.stamp = ev.ts;
                    // evQueue.header.frame_id = i;
                    // evQueue.height = sensor_height_;
                    // evQueue.width = sensor_width_;
                    
                    //cout<<"event size=="<<evPtr->events.size()<<endl;
                    events_left_pub_.publish(evPtr);
                    evPtr->events.clear(); 
                    //publishEvents();
                   // bag.write(topic, ev.ts, evQueue);
                    //evQueue.events.clear();
                //}
            
            //bag.close();
             if(t.size() != numev_per_stream){              
                 cout<<"Final bag file has been written."<<endl;
                 break;
             }
            hslab_offset+=numev_per_stream;
            frame_id++;
            //r.sleep();
        }
        
    }
      // end of try block
    // catch failure caused by the H5File operations
    catch( FileIException error )
    {
        error.printErrorStack();
        return ;
    }
    // catch failure caused by the DataSet operations
    catch( DataSetIException error )
    {
        error.printErrorStack();
        return ;
    }
    // catch failure caused by the DataSpace operations
    catch( DataSpaceIException error )
    {
        error.printErrorStack();
        return ;
    }
    // catch failure caused by the DataSpace operations
    catch( DataTypeIException error )
    {
        error.printErrorStack();
        return ;
    }
    catch(rosbag::BagException error){
        cout<<error.what()<<endl;
        return ;
    }
    
    return ;  // successfully terminated
}
void eventProvider::publishEvents(dvs_msgs::EventArrayPtr &msg){
    
    cout<<"PUBLISHING EVENTS"<<endl;
}