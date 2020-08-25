#pragma warning(disable : 4996)
#include <k4arecord/playback.h>
#include <k4a/k4a.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>

#include <fstream>
#include <sstream>

#include <vector>

void depth_point_cloud(k4a_playback_t playback,k4a_image_t depth, int i);
static void create_xy_table(const k4a_calibration_t* calibration, k4a_image_t xy_table);
static void generate_point_cloud(const k4a_image_t depth_image,
	const k4a_image_t xy_table,
	k4a_image_t point_cloud,
	int* point_count);
static void write_point_cloud(const char* file_name, const k4a_image_t point_cloud, int point_count);


using namespace std;

struct color_point_t
{
	int16_t xyz[3];
	uint8_t rgb[3];
};



int main()
{
	k4a_capture_t capture = NULL;
	k4a_stream_result_t result = K4A_STREAM_RESULT_SUCCEEDED;

	// Play Back object
	k4a_playback_t playback_handle = NULL;

	if (k4a_playback_open("1.mkv", &playback_handle) != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to open recording\n");
		return 1;
	}
	else
	{
		printf("Open Successfully");
	}

	
	int i = 0;
	while (result == K4A_STREAM_RESULT_SUCCEEDED)
	{
		//Read Every Capture

		result = k4a_playback_get_next_capture(playback_handle, &capture);
		k4a_image_t depth_image = NULL;

		//Operation on Each capture
		if (result == K4A_STREAM_RESULT_SUCCEEDED)
		{
			// Process capture here
			depth_image = k4a_capture_get_depth_image(capture);
			if (depth_image!= NULL)
			{
				
				k4a_image_t color_image = NULL;
				color_image = k4a_capture_get_color_image(capture);
				if (color_image != NULL)
				{
					depth_point_cloud(playback_handle,depth_image,i);
					i = i + 1;
					
				}
				k4a_image_release(depth_image);
				k4a_image_release(color_image);

			}
		}
		else if (result == K4A_STREAM_RESULT_EOF)
		{
			// End of file reached
			break;
		}
	}



	//Close PlayBack
	k4a_playback_close(playback_handle);
	return 1;
}

void depth_point_cloud(k4a_playback_t playback,k4a_image_t depth, int i)
{
	k4a_calibration_t calibration;
	k4a_transformation_t transformation = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration))
	{
		printf("Failed to get calibration\n");
		
	}

	transformation = k4a_transformation_create(&calibration);

	k4a_image_t xy_table = NULL;
	k4a_image_t point_cloud = NULL;
	int point_count = 0;
	int depth_width, depth_height;
	depth_width = k4a_image_get_width_pixels(depth);
	depth_height = k4a_image_get_height_pixels(depth);

	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		calibration.depth_camera_calibration.resolution_width,
		calibration.depth_camera_calibration.resolution_height,
		calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
		&xy_table);
	create_xy_table(&calibration, xy_table);

	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		calibration.depth_camera_calibration.resolution_width,
		calibration.depth_camera_calibration.resolution_height,
		calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
		&point_cloud);


	generate_point_cloud(depth, xy_table, point_cloud, &point_count);

	char file_name[20] = "point_cloud";
	
	char snum[5];

	itoa(i, snum, 10);
	
	strcat(file_name,snum);
	
	char format_pass[6] = ".ply";
	strcat(file_name, format_pass);
	printf("%s\n", file_name);

	write_point_cloud(file_name, point_cloud, point_count);

	k4a_image_release(xy_table);
	k4a_image_release(point_cloud);
}

static void create_xy_table(const k4a_calibration_t* calibration, k4a_image_t xy_table)
{
	k4a_float2_t* table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);

	int width = calibration->depth_camera_calibration.resolution_width;
	int height = calibration->depth_camera_calibration.resolution_height;

	k4a_float2_t p;
	k4a_float3_t ray;
	int valid;

	for (int y = 0, idx = 0; y < height; y++)
	{
		p.xy.y = (float)y;
		for (int x = 0; x < width; x++, idx++)
		{
			p.xy.x = (float)x;

			k4a_calibration_2d_to_3d(
				calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

			if (valid)
			{
				table_data[idx].xy.x = ray.xyz.x;
				table_data[idx].xy.y = ray.xyz.y;
			}
			else
			{
				table_data[idx].xy.x = nanf("");
				table_data[idx].xy.y = nanf("");
			}
			
		}
	}
}

static void generate_point_cloud(const k4a_image_t depth_image,
	const k4a_image_t xy_table,
	k4a_image_t point_cloud,
	int* point_count)
{
	int width = k4a_image_get_width_pixels(depth_image);
	int height = k4a_image_get_height_pixels(depth_image);

	uint16_t* depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depth_image);
	k4a_float2_t* xy_table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);
	k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

	*point_count = 0;
	for (int i = 0; i < width * height; i++)
	{
		if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
		{
			point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
			point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
			point_cloud_data[i].xyz.z = (float)depth_data[i];
			(*point_count)++;
		}
		else
		{
			point_cloud_data[i].xyz.x = nanf("");
			point_cloud_data[i].xyz.y = nanf("");
			point_cloud_data[i].xyz.z = nanf("");
		}
	}
}

static void write_point_cloud(const char* file_name, const k4a_image_t point_cloud, int point_count)
{
	int width = k4a_image_get_width_pixels(point_cloud);
	int height = k4a_image_get_height_pixels(point_cloud);

	k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

	// save to the ply file
	std::ofstream ofs(file_name); // text mode first
	ofs << "ply" << std::endl;
	ofs << "format ascii 1.0" << std::endl;
	ofs << "element vertex"
		<< " " << point_count << std::endl;
	ofs << "property float x" << std::endl;
	ofs << "property float y" << std::endl;
	ofs << "property float z" << std::endl;
	ofs << "end_header" << std::endl;
	ofs.close();

	std::stringstream ss;
	for (int i = 0; i < width * height; i++)
	{
		if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
		{
			continue;
		}

		ss << (float)point_cloud_data[i].xyz.x << " " << (float)point_cloud_data[i].xyz.y << " "
			<< (float)point_cloud_data[i].xyz.z << std::endl;
	}

	std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
	ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}
