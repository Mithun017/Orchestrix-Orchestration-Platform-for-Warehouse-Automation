from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .wms import router as wms_router
from .wcs import router as wcs_router
from .robots import router as robots_router

app = FastAPI(
    title="ORCHESTRIX",
    description="Warehouse Orchestration Platform Mock",
    version="1.0.0"
)

# CORS (Allow Frontend to connect)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], # In production, specify exact origin
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include Routers
app.include_router(wms_router)
app.include_router(wcs_router)
app.include_router(robots_router)

from .map_api import router as map_router
app.include_router(map_router)

@app.get("/")
async def root():
    return {"message": "ORCHESTRIX System Online"}
