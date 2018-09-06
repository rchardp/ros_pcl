import pclpy.pcdio

cloud = loadPCDFile('inputCloud.pcd')

# aplicarle un filtro a la nube
# rcloud = filter

ans = savePCDFile( rcloud, 'outputCloud.pcd' )

print( 'Exito al guardar' if ans else 'Ocurrió un error al guardar' )
