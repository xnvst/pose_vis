#include "pose_vis.h"

void PoseVis::CreatePoseFromData(std::string bpos_data_path)
{
    std::cout << __FUNCTION__ << std::endl;

    fc_bpos_recoder_t temp_bpos;
    size_t data_size = sizeof(fc_bpos_recoder_t);

    FILE * pFile;
    long total_size;
    char * buffer;
    size_t result;
    if ((pFile = fopen(bpos_data_path.c_str(), "rb")) == NULL)
    {
        std::cout << "Error! opening file" << std::endl;
        return;
    }
    fseek(pFile, 0, SEEK_END);
    total_size = ftell(pFile);
    rewind(pFile);
    std::cout << "pose file size = " << total_size << std::endl;

    size_t size = 0;
    int packet_count = 0;
    while (size < total_size)
    {
        memset(&temp_bpos, 0, data_size);
        size_t read_byte = fread(&temp_bpos, 1, data_size, pFile);
        packet_count++;

        Pose temp_pose(
                temp_bpos.latitude,
                temp_bpos.longitude,
                temp_bpos.altitue,
                temp_bpos.qw,
                temp_bpos.qx,
                temp_bpos.qy,
                temp_bpos.qz,
                temp_bpos.Time
                );

        if(std::isnan(temp_bpos.Week)
                || std::isnan(temp_bpos.Time)
                || std::isnan(temp_bpos.latitude)
                || std::isnan(temp_bpos.longitude)
                || std::isnan(temp_bpos.altitue)
                || std::isnan(temp_bpos.n_posistion_rmse)
                || std::isnan(temp_bpos.e_posistion_rmse)
                || std::isnan(temp_bpos.d_posistion_rmse)
                || std::isnan(temp_bpos.qw)
                || std::isnan(temp_bpos.qx)
                || std::isnan(temp_bpos.qy)
                || std::isnan(temp_bpos.qz)
                || std::isnan(temp_bpos.roll_rmes)
                || std::isnan(temp_bpos.pitch_rmse)
                || std::isnan(temp_bpos.heading_rmse)
          )
        {
            std::cout<<"\n[Error]: Found invalid bpos data:"<<std::endl;
        }

        _fc_pos_vec.push_back(temp_pose);

        size += read_byte;
    }

    return;
}