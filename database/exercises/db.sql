--
-- PostgreSQL database dump
--

-- Dumped from database version 17.0 (Debian 17.0-1.pgdg120+1)
-- Dumped by pg_dump version 17.0 (Debian 17.0-1.pgdg120+1)

SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET transaction_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SELECT pg_catalog.set_config('search_path', '', false);
SET check_function_bodies = false;
SET xmloption = content;
SET client_min_messages = warning;
SET row_security = off;

SET default_tablespace = '';

SET default_table_access_method = heap;

--
-- Name: exercises; Type: TABLE; Schema: public; Owner: user-dev
--

CREATE TABLE public.exercises (
    id integer NOT NULL,
    exercise_id character varying(40) NOT NULL,
    name character varying(40) NOT NULL,
    description character varying(400) NOT NULL,
    tags character varying(2000) NOT NULL,
    status character varying(20) NOT NULL,
    template character varying(200) NOT NULL
);


ALTER TABLE public.exercises OWNER TO "user-dev";

--
-- Name: exercises_id_seq; Type: SEQUENCE; Schema: public; Owner: user-dev
--

ALTER TABLE public.exercises ALTER COLUMN id ADD GENERATED BY DEFAULT AS IDENTITY (
    SEQUENCE NAME public.exercises_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1
);


--
-- Name: exercises_universes; Type: TABLE; Schema: public; Owner: user-dev
--

CREATE TABLE public.exercises_universes (
    id bigint NOT NULL,
    exercise_id bigint NOT NULL,
    universe_id bigint NOT NULL
);


ALTER TABLE public.exercises_universes OWNER TO "user-dev";

--
-- Name: exercises_universes_id_seq; Type: SEQUENCE; Schema: public; Owner: user-dev
--

ALTER TABLE public.exercises_universes ALTER COLUMN id ADD GENERATED BY DEFAULT AS IDENTITY (
    SEQUENCE NAME public.exercises_universes_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1
);


--
-- Data for Name: exercises; Type: TABLE DATA; Schema: public; Owner: user-dev
--

COPY public.exercises (id, exercise_id, name, description, tags, status, template) FROM stdin;
1	follow_line	Follow Line	The goal of this exercise is to perform a PID reactive control capable of following the line painted on the racing circuit.	{"tags": ["ROS2","AUTONOMOUS DRIVING"]}	ACTIVE	RoboticsAcademy/exercises/static/exercises/follow_line/python_template/
2	vacuum_cleaner	Vacum Cleaner	Vacuum Cleaner exercise using React and RAM	{"tags": "ROS2"}	ACTIVE	RoboticsAcademy/exercises/static/exercises/vacuum_cleaner/python_template/
3	autoparking	Autoparking	Autoparking exercise testing	{"tags": ["AUTONOMOUS DRIVING","SERVICE ROBOTS","ROS2"] }	ACTIVE	RoboticsAcademy/exercises/static/exercises/autoparking/python_template/
4	follow_person	Follow Person	Follow a person with kobuki robot	{"tags": "ROS2"}	ACTIVE	RoboticsAcademy/exercises/static/exercises/follow_person/python_template/
5	vacuum_cleaner_loc	Localized Vacuum Cleaner	Localiized vauum clenaer	{"tags": ["ROS2", "MOBILE ROBOTS", "SERVICE ROBOTS"]}	ACTIVE	RoboticsAcademy/exercises/static/exercises/vacuum_cleaner_loc/python_template/
6	global_navigation	Global Navigation	Global navigation exercise using REACT and RAM	{"tags": "ROS2"}	ACTIVE	RoboticsAcademy/exercises/static/exercises/global_navigation/python_template/
7	rescue_people	Rescue People	Rescue People exercise using React and RAM	{"tags": "ROS2"}	ACTIVE	RoboticsAcademy/exercises/static/exercises/rescue_people/python_template/
8	obstacle_avoidance	Obstacle Avoidance	Obstacle Avoidance exercise using React and RAM	{"tags": ["ROS2","AUTONOMOUS DRIVING"]}	ACTIVE	RoboticsAcademy/exercises/static/exercises/obstacle_avoidance/python_template/
9	3d_reconstruction	3D Reconstruction	3D Reconstruction exercise using React and RAM	{"tags": ["ROS2","COMPUTER VISION"]}	ACTIVE	RoboticsAcademy/exercises/static/exercises/3d_reconstruction/python_template/
10	amazon_warehouse	Amazon Warehouse	Control an amazon-like robot to organize a warehouse	{"tags": "ROS2"}	ACTIVE	RoboticsAcademy/exercises/static/exercises/amazon_warehouse/python_template/
11	montecarlo_laser_loc	Montecarlo Laser Loc	Calculate the position of the robot based on the	{"tags": "ROS2"}	ACTIVE	RoboticsAcademy/exercises/static/exercises/montecarlo_laser_loc/python_template/
12	montecarlo_visual_loc	Montecarlo Visual Loc	Calculate the position of the robot based on the	{"tags": "ROS2"}	ACTIVE	RoboticsAcademy/exercises/static/exercises/montecarlo_visual_loc/python_template/
\.


--
-- Data for Name: exercises_universes; Type: TABLE DATA; Schema: public; Owner: user-dev
--

COPY public.exercises_universes (id, exercise_id, universe_id) FROM stdin;
1	2	1
2	5	1
3	6	2
4	7	3
5	10	4
6	1	5
7	1	6
8	4	7
9	4	8
10	3	9
11	11	1
12	8	10
13	9	12
14	3	13
15	3	14
16	3	15
17	3	16
18	10	17
19	10	18
20	10	19
21	1	20
22	1	21
23	1	22
24	1	23
25	1	24
26	1	25
27	12	27
\.


--
-- Name: exercises_id_seq; Type: SEQUENCE SET; Schema: public; Owner: user-dev
--

SELECT pg_catalog.setval('public.exercises_id_seq', 1, false);


--
-- Name: exercises_universes_id_seq; Type: SEQUENCE SET; Schema: public; Owner: user-dev
--

SELECT pg_catalog.setval('public.exercises_universes_id_seq', 1, false);


--
-- Name: exercises exercises_exercise_id_key; Type: CONSTRAINT; Schema: public; Owner: user-dev
--

ALTER TABLE ONLY public.exercises
    ADD CONSTRAINT exercises_exercise_id_key UNIQUE (exercise_id);


--
-- Name: exercises exercises_name_key; Type: CONSTRAINT; Schema: public; Owner: user-dev
--

ALTER TABLE ONLY public.exercises
    ADD CONSTRAINT exercises_name_key UNIQUE (name);


--
-- Name: exercises exercises_pkey; Type: CONSTRAINT; Schema: public; Owner: user-dev
--

ALTER TABLE ONLY public.exercises
    ADD CONSTRAINT exercises_pkey PRIMARY KEY (id);


--
-- Name: exercises_universes exercises_universes_exercise_id_universe_id_155bee4e_uniq; Type: CONSTRAINT; Schema: public; Owner: user-dev
--

ALTER TABLE ONLY public.exercises_universes
    ADD CONSTRAINT exercises_universes_exercise_id_universe_id_155bee4e_uniq UNIQUE (exercise_id, universe_id);


--
-- Name: exercises_universes exercises_universes_pkey; Type: CONSTRAINT; Schema: public; Owner: user-dev
--

ALTER TABLE ONLY public.exercises_universes
    ADD CONSTRAINT exercises_universes_pkey PRIMARY KEY (id);


--
-- Name: exercises_exercise_id_c21d158f_like; Type: INDEX; Schema: public; Owner: user-dev
--

CREATE INDEX exercises_exercise_id_c21d158f_like ON public.exercises USING btree (exercise_id varchar_pattern_ops);


--
-- Name: exercises_name_d52987aa_like; Type: INDEX; Schema: public; Owner: user-dev
--

CREATE INDEX exercises_name_d52987aa_like ON public.exercises USING btree (name varchar_pattern_ops);


--
-- Name: exercises_universes_exercise_id_e4e984bc; Type: INDEX; Schema: public; Owner: user-dev
--

CREATE INDEX exercises_universes_exercise_id_e4e984bc ON public.exercises_universes USING btree (exercise_id);


--
-- Name: exercises_universes_universe_id_13c30e27; Type: INDEX; Schema: public; Owner: user-dev
--

CREATE INDEX exercises_universes_universe_id_13c30e27 ON public.exercises_universes USING btree (universe_id);


--
-- Name: exercises_universes exercises_universes_exercise_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: user-dev
--

ALTER TABLE ONLY public.exercises_universes
    ADD CONSTRAINT exercises_universes_exercise_id_fkey FOREIGN KEY (exercise_id) REFERENCES public.exercises(id);


--
-- Name: exercises_universes exercises_w_exercise_id_e4e984bc_fk_exercises; Type: FK CONSTRAINT; Schema: public; Owner: user-dev
--

ALTER TABLE ONLY public.exercises_universes
    ADD CONSTRAINT exercises_w_exercise_id_e4e984bc_fk_exercises FOREIGN KEY (exercise_id) REFERENCES public.exercises(id) DEFERRABLE INITIALLY DEFERRED;


--
-- PostgreSQL database dump complete
--

