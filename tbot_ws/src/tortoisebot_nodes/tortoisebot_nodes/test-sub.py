from launch.substitutions import AndSubstitution,TextSubstitution

path=AndSubstitution([TextSubstitution(text='home'),TextSubstitution(text='WIFI')])
print(path)