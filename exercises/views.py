import json
import mimetypes
import os
import shutil
import tempfile
import subprocess
import zipfile
import pylint as lint
from django.shortcuts import render
from django.http import HttpResponse
from django.conf import settings
from django.views.decorators.csrf import csrf_exempt
from django.http import JsonResponse
from .models import Exercise
from rest_framework.response import Response
from rest_framework import status


def get_python_code(request):
    python_code = request.GET.get('python_code', None)
    print("A", python_code)
    if not python_code:
        body_unicode = request.body.decode('utf-8')
        body_unicode = body_unicode[0:18] + body_unicode[18: len(body_unicode) - 2].replace('"',
                                                                                            "'") + body_unicode[-2:]

        body = json.loads(body_unicode, strict=False)

        python_code = body['python_code']
        print("B")
        print(python_code)
    python_code = python_code.lstrip('\\').lstrip('"')
    python_code = python_code.replace('\\n', '\n')
    python_code = python_code.replace('\\"', '"').replace("\\'", "'")
    return python_code

@csrf_exempt
def ros_version(request):    
    output = subprocess.check_output(['bash', '-c', 'echo $ROS_VERSION'])
    output_str = output.decode('utf-8')
    version = output_str[0]
    data = {'version': version}
    return JsonResponse(data)

@csrf_exempt
def launch_files(request, exercise_id):
    exercise = Exercise.objects.get(exercise_id=exercise_id)
    return JsonResponse(data)

# TODO: Too many hardcoded strings, review
def index(request):
    exercises = Exercise.objects.all()
    context = {"exercises": exercises}
    return render(request, 'exercises/RoboticsAcademy.html', context)


def load_exercise(request, exercise_id):
    exercise = Exercise.objects.get(exercise_id=exercise_id)
    print(exercise.worlds)
    return render(request, 'exercises/' + exercise_id + '/exercise.html', exercise.context)


def request_code(request, exercise_id):
    difficulty = request.GET.get('diff')
    path = f'/exercises/static/exercises/{exercise_id}/assets/{difficulty}.py'
    path = str(settings.BASE_DIR) + path
    print('PATH: ', path)
    with open(path, encoding='utf-8') as file:
        data = file.read().replace('\\n', '\n')

    print(data)

    if difficulty is not None:
        print('EXERCISE: ', exercise_id, 'DIFFICULTY: ', difficulty)
        return HttpResponse(data, content_type="text/plain")

@csrf_exempt  
def user_code_zip(request, exercise_id):
        
    code = json.loads(request.body)["code"]
    working_folder = "/tmp/ra"

    exercise_path = os.path.join(settings.BASE_DIR, f"exercises/static/exercises/{exercise_id}/python_template/ros2_humble")
    common_path = os.path.join(settings.BASE_DIR, "common")
    console_interfaces_path = os.path.join(common_path, "console_interfaces/console_interfaces")
    gui_interfaces_path = os.path.join(common_path, "gui_interfaces/gui_interfaces")
    hal_interfaces_path = os.path.join(common_path, "hal_interfaces/hal_interfaces")
    user_path = os.path.join(working_folder, "academy.py")

    console_working_path = os.path.join(working_folder, "console_interfaces")
    gui_working_path = os.path.join(working_folder, "gui_interfaces")
    hal_working_path = os.path.join(working_folder, "hal_interfaces")

    try:
        # 1. Create the working folder
        if os.path.exists(working_folder):
            shutil.rmtree(working_folder)
        os.mkdir(working_folder)

        # 2. Copy necessary files
        shutil.copytree(console_interfaces_path, console_working_path, dirs_exist_ok=True)
        shutil.copytree(gui_interfaces_path, gui_working_path, dirs_exist_ok=True)
        shutil.copytree(hal_interfaces_path, hal_working_path, dirs_exist_ok=True)
        shutil.copytree(exercise_path, working_folder, dirs_exist_ok=True)

        # 3. Copy user code
        f = open(user_path, "w")
        f.write(code)
        f.close()

        # 3. Generate the zip
        zip_path = working_folder + ".zip"
        with zipfile.ZipFile(zip_path, "w") as zipf:
            for root, dirs, files in os.walk(working_folder):
                for file in files:
                    zipf.write(
                        os.path.join(root, file),
                        os.path.relpath(os.path.join(root, file), working_folder),
                    )

        # 4. Return the zip
        zip_file = open(zip_path, "rb")
        mime_type, _ = mimetypes.guess_type(zip_path)
        response = HttpResponse(zip_file, content_type=mime_type)
        response["Content-Disposition"] = (
            f"attachment; filename={os.path.basename(zip_path)}"
        )

        return response
    except Exception as e:
        return Response({"success": False, "message": str(e)}, status=status.HTTP_400_BAD_REQUEST)
