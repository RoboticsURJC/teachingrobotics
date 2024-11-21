#!/bin/bash

set -e
set -u

psql -U user-dev -d academy_db < /universes/universes.sql
psql -U user-dev -d academy_db < /docker-entrypoint-initdb.d/exercises/db.sql

alias saveDb='./scripts/saveDb.sh'