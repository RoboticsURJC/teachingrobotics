from django.contrib import admin
from django.db import models
from .models import Exercise, Universe


# Register your models here.

admin.site.register(Exercise)
admin.site.register(Universe)
