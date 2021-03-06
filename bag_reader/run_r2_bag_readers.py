import func_raven_bag_reader as reader

record_name = '_rand_1200_for_xyz'
#record_name = ''

bag_path = 'bag_files/recorded_trajs_xyz'
result_path = 'result_files/recorded_trajs_xyz'

bag_name = 'record_ext_jenc' + record_name + '.bag'
result_name = bag_name[:-3] + 'csv'

reader.read_bag_ext_joint_encoder(bag_name = bag_path + '/' + bag_name, output_name = result_path + '/' + result_name)

# --------------------------------------------------------------------------

bag_name = 'record_CRTKmeasurejs' + record_name + '.bag'
result_name = bag_name[:-3] + 'csv'

reader.read_bag_CRTK_measured_js(bag_name = bag_path + '/' + bag_name, output_name = result_path + '/' + result_name)

# --------------------------------------------------------------------------

bag_name = 'record_ravenstate' + record_name + '.bag'
result_name = bag_name[:-3] + 'csv'

reader.read_bag_ravenstate(bag_name = bag_path + '/' + bag_name, output_name = result_path + '/' + result_name)
