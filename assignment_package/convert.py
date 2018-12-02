import sys, os
for f in os.listdir("."):
    if f.endswith('.geo'):
        geo = f.replace('.geo', '.bgeo')
        if os.path.isfile(geo):
            continue
        
        cmd = 'gconvert %s %s' % (f, geo)
        print cmd
        if os.system(cmd):
             print 'Error running command'
             break
