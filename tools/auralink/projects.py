# Note: we need to pick a place to save the projects data.  Ultimately
# there are better places to choose, but for now we will stick it in
# the same place as the code itself.

import json
import os

dir = os.path.dirname(__file__)

projects_file = os.path.join(dir, 'projects.json')
print('map projects saved here:', projects_file)

def load():
    if not os.path.isfile( projects_file ):
        return ""
    try:
        f = open(projects_file, 'r')
        stream = f.read();
        f.close()
        return stream
    except:
        print('error reading:', projects_file)
        return ""

def save( json_dict ):
    try:
        f = open(projects_file, 'w')
        json.dump(json_dict, f, indent=4, sort_keys=True)
        f.close()
    except:
        print('error writing:', projects_file, str(sys.exc_info()[1]))

# update or create project by name
def update_name( new_json ):
    json_str = load()
    current_projects = json.loads(json_str)
    new = json.loads( new_json )
    name = new["name"]
    areas = new["areas"]
    current_projects[name] = areas
    current_projects["projects_magic"] = True
    save(current_projects)
    
# delete project
def delete_name( name ):
    print('delete project:', name)
    json_str = load()
    current_projects = json.loads(json_str)
    current_projects.pop(name, None)
    save(current_projects)
