#!/bin/bash

echo "Saving databases"
pg_dump -U user-dev -d academy_db --exclude-table public.universes --exclude-table public.exercises --exclude-table public.exercises_universes > /docker-entrypoint-initdb.d/django_auth_dump.sql
pg_dump -U user-dev -d academy_db --table public.exercises --table public.exercises_universes > /docker-entrypoint-initdb.d/exercises/dump.sql
pg_dump -U user-dev -d academy_db --table public.universes > /docker-entrypoint-initdb.d/universes.sql