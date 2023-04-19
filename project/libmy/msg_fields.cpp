for (int i = 0; i < (int)laserCloudMsg.fields.size(); ++i)
{
  // traverse fields to fint out whether exist a varible named in "ring"
  // ring indicates point locate in which scan(horizontal)
  std::cout << laserCloudMsg.fields[i].name << std::endl;
  std::cout << laserCloudMsg.fields[i] << std::endl;
}
