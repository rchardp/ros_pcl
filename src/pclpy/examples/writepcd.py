from pclpy.pcdio import loadPCDFile, savePCDFile

cloud = loadPCDFile('inputCloud.pcd')

ans = savePCDFile( cloud, 'outputCloud.pcd' )

print( 'Exito al guardar' if ans else 'Ocurrió un error al guardar' )
