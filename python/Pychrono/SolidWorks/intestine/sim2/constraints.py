# if i != 12:
#     my_link2 = my_system.SearchBody('robo_leg_link-' + str(i + 1) + '/link1-1')
#     if not my_link2:
#         sys.exit('Error: cannot find link2 from its name in the C::E system!')
#     spring_marker = my_link.SearchMarker('Spring_marker2')
#     spring = chrono.ChLinkTSDA()
#     spring.SetName('spring' + str(i))
#     spring.Initialize(my_link, my_link2, False,
#                       chrono.ChVectorD(spring_marker.GetAbsCoord().pos.x, spring_marker.GetAbsCoord().pos.y,
#                                        spring_marker.GetAbsCoord().pos.z),
#                       chrono.ChVectorD(spring_marker.GetAbsCoord().pos.x, spring_marker.GetAbsCoord().pos.y,
#                                        spring_marker.GetAbsCoord().pos.z))
#     spring.SetSpringCoefficient(2)
#     spring.SetDampingCoefficient(0.1)
#     # Visualization element
#     # col1 = chrono.ChColorAsset()
#     # col1.SetColor(chrono.ChColor(0, 1, 0))  # Green
#     # spring.AddAsset(col1)
#     my_system.AddLink(spring)
#
# elif i == 12:
#     my_link2 = my_system.SearchBody('robo_leg_link-' + str(1) + '/link1-1')
#     if not my_link2:
#         sys.exit('Error: cannot find link2 from its name in the C::E system!')
#     spring_marker = my_link.SearchMarker('Spring_marker2')
#     spring = chrono.ChLinkTSDA()
#     spring.SetName('spring' + str(i))
#     spring.Initialize(my_link, my_link2, False,
#                       chrono.ChVectorD(spring_marker.GetAbsCoord().pos.x, spring_marker.GetAbsCoord().pos.y,
#                                        spring_marker.GetAbsCoord().pos.z),
#                       chrono.ChVectorD(spring_marker.GetAbsCoord().pos.x, spring_marker.GetAbsCoord().pos.y,
#                                        spring_marker.GetAbsCoord().pos.z))
#     spring.SetSpringCoefficient(2)
#     spring.SetDampingCoefficient(0.1)
#     # # Visualization element
#     # col1 = chrono.ChColorAsset()
#     # col1.SetColor(chrono.ChColor(0, 1, 0))  # Green
#     # spring.AddAsset(col1)
#     my_system.AddLink(spring)