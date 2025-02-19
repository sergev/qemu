/* AUTOMATICALLY GENERATED, DO NOT MODIFY */

/*
 * schema-defined QAPI function prototypes
 *
 * Copyright IBM, Corp. 2011
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 */

#ifndef QGA_QMP_COMMANDS_H
#define QGA_QMP_COMMANDS_H

#include "qga-qapi-types.h"
#include "qapi/qmp/qdict.h"
#include "qapi/error.h"

int64_t qmp_guest_sync_delimited(int64_t id, Error **errp);
int64_t qmp_guest_sync(int64_t id, Error **errp);
void qmp_guest_ping(Error **errp);
int64_t qmp_guest_get_time(Error **errp);
void qmp_guest_set_time(bool has_time, int64_t time, Error **errp);
GuestAgentInfo *qmp_guest_info(Error **errp);
void qmp_guest_shutdown(bool has_mode, const char *mode, Error **errp);
int64_t qmp_guest_file_open(const char *path, bool has_mode, const char *mode, Error **errp);
void qmp_guest_file_close(int64_t handle, Error **errp);
GuestFileRead *qmp_guest_file_read(int64_t handle, bool has_count, int64_t count, Error **errp);
GuestFileWrite *qmp_guest_file_write(int64_t handle, const char *buf_b64, bool has_count, int64_t count, Error **errp);
GuestFileSeek *qmp_guest_file_seek(int64_t handle, int64_t offset, int64_t whence, Error **errp);
void qmp_guest_file_flush(int64_t handle, Error **errp);
GuestFsfreezeStatus qmp_guest_fsfreeze_status(Error **errp);
int64_t qmp_guest_fsfreeze_freeze(Error **errp);
int64_t qmp_guest_fsfreeze_freeze_list(bool has_mountpoints, strList *mountpoints, Error **errp);
int64_t qmp_guest_fsfreeze_thaw(Error **errp);
void qmp_guest_fstrim(bool has_minimum, int64_t minimum, Error **errp);
void qmp_guest_suspend_disk(Error **errp);
void qmp_guest_suspend_ram(Error **errp);
void qmp_guest_suspend_hybrid(Error **errp);
GuestNetworkInterfaceList *qmp_guest_network_get_interfaces(Error **errp);
GuestLogicalProcessorList *qmp_guest_get_vcpus(Error **errp);
int64_t qmp_guest_set_vcpus(GuestLogicalProcessorList *vcpus, Error **errp);
GuestFilesystemInfoList *qmp_guest_get_fsinfo(Error **errp);
void qmp_guest_set_user_password(const char *username, const char *password, bool crypted, Error **errp);
GuestMemoryBlockList *qmp_guest_get_memory_blocks(Error **errp);
GuestMemoryBlockResponseList *qmp_guest_set_memory_blocks(GuestMemoryBlockList *mem_blks, Error **errp);
GuestMemoryBlockInfo *qmp_guest_get_memory_block_info(Error **errp);

#endif
