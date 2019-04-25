# goose
duck, duck

```
docker stop <container name>
docker rm <container name>
docker -H <host> build -t <version tag> .
docker -H <host> run -[d]it --privileged --network=host [--name] -v /data:/data <version tag>
```
To start mqtt broker:
`docker -H 192.168.1.70 run -dit -p 1883:1883 -p 9001:9001 eclipse-mosquitto`
