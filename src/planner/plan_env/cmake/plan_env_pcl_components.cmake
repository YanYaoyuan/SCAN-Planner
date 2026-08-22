# pcl_conversions exports PCL without component constraints.  On headless
# targets that makes downstream users search for optional visualization/VTK
# modules even though plan_env only uses common, io and filters.  Resolve the
# production component set before ament evaluates transitive dependencies.
if(NOT PCL_FOUND)
  find_package(PCL REQUIRED COMPONENTS common io filters)
endif()
